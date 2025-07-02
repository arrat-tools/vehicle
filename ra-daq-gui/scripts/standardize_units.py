#!/usr/bin/env python
import os, json, sys, pyproj, cv2, argparse, csv, glob, math, rosbag, shutil
import numpy as np
from scipy import interpolate

IMG_DEPTH_TIME_TOL = 0.05

def get_min(input_list):
    # min_value, min_index = get_min(np.abs(np.array(s_time)-item['event_start_time']))
    min_value = np.amin(np.array(input_list))
    return min_value, np.where(input_list == min_value)[0][0]

def fix_diff_heading(input_angle):
	# method 1
    # # fix diff heading
    # for i, item in enumerate(h_d):
    # 	if i == 0:
    # 		output = [h_d[0]]
    # 	else:
    # 		dh = h_d[i] - h_d[i-1]
    # 		if dh > 180:
    # 			dh = dh - 360
    # 		if dh < -180:
    # 			dh = dh + 360
    # 		output.append(output[i-1] + dh)
    # # interp
    # heading = np.interp(timestamps, np.array(timestamps)[i_d], output)
    # # convert to azimuth
    # azimuth = 90.0 - np.array(heading)

    # method 2
    # convert to azimuth
    azimuth_d = 90.0 - input_angle
    # fix diff azimuth
    for i, item in enumerate(azimuth_d):
        if i == 0:
            output = [azimuth_d[0]]
        else:
            dh = azimuth_d[i] - azimuth_d[i-1]
            if dh > 180:
                dh = dh - 360
            if dh < -180:
                dh = dh + 360
            output.append(output[i-1] + dh)
    return output

def find_nearest(x_raw, y_raw, x_fit, y_fit):
    x_near, y_near, i_near = [], [], []
    for i in range(0, len(x_raw)):
        x, y = x_raw[i], y_raw[i]
        dist_all = np.hypot(x_fit-x, y_fit-y)
        min_value, min_index = get_min(np.abs(np.array(dist_all)))
        x_near.append(x_fit[min_index])
        y_near.append(y_fit[min_index])
        i_near.append(min_index)
    return x_near, y_near, i_near


def downsample_path(x, y, spacing=1.0):
	x_downsampled, y_downsampled = [], []
	coords_downsampled = []
	indices_downsampled = []

	for i in range(0, len(x)):                
		if i == 0: # first point so just accept whatever gps position and init pp
			x_downsampled.append(x[i])
			y_downsampled.append(y[i])
			x_prev, y_prev = x[i], y[i]
			coords_downsampled.append((x[i], y[i]))
			indices_downsampled.append(i)

		elif i == len(x)-1: # if last point, just append
			x_downsampled.append(x[i])
			y_downsampled.append(y[i])
			coords_downsampled.append((x[i], y[i]))
			indices_downsampled.append(i)
		
		else: # if not first or last then check delta path
			dx = np.hypot(x[i]-x_prev, y[i]-y_prev)
			if dx >= spacing: # append if path spacing exceeds desired spacing
				x_downsampled.append(x[i])
				y_downsampled.append(y[i])
				coords_downsampled.append((x[i], y[i]))
				indices_downsampled.append(i)
				x_prev, y_prev = x[i], y[i]

	return x_downsampled, y_downsampled, coords_downsampled, indices_downsampled

def spline_curvature(tck, unew):
    dx, dy = interpolate.splev(unew, tck, der=1)
    ddx, ddy = interpolate.splev(unew, tck, der=2)
    K = (dx * ddy - dy * ddx) / ((dx ** 2 + dy ** 2) ** (3 / 2))
    hdg = np.arctan2(dy, dx)
    return K, hdg

def spline_fit(x_raw, y_raw, ds, smoothing=None):
    if smoothing is not None:
        tck, u = interpolate.splprep([x_raw, y_raw], s=smoothing)
    else:
        tck, u = interpolate.splprep([x_raw, y_raw])
    u_fit = np.arange(0, 1+ds, ds)
    out = interpolate.splev(u_fit, tck)
    x_fit, y_fit = out[0], out[1]
    k_fit, h_fit = spline_curvature(tck, u_fit)
    return x_fit, y_fit, k_fit, h_fit

def smooth_waypoints(x_raw, y_raw, dx_frame=1, path_variance=0.1):
    Nt = len(x_raw)
    ds = 1/(Nt*dx_frame) # n points in spline fit are approx 1 ft apart
    x_fit, y_fit, k_fit, h_fit = spline_fit(x_raw, y_raw, ds, smoothing=(path_variance**2)*len(x_raw))
    return x_fit, y_fit, h_fit

def heading_from_path(east, north, timestamps):
	# downsample
    e_d, n_d, coords_dd, i_d = downsample_path(east, north, spacing=5.0)
    # smooth
    SMOOTH_FACTOR, SMOOTH_VARIANCE = 10, 0.1
    e_fit, n_fit, h_fit = smooth_waypoints(e_d, n_d, dx_frame=SMOOTH_FACTOR, path_variance=SMOOTH_VARIANCE)
    # nearest
    x_near, y_near, i_near = find_nearest(e_d, n_d, e_fit, n_fit)
    # heading at downsampled
    h_d = 180/np.pi*np.array(h_fit)[i_near]
    # fix diff heading
    azimuth = fix_diff_heading(h_d)
    # interp
    azimuth = np.interp(timestamps, np.array(timestamps)[i_d], azimuth)
    azimuth = azimuth % 360.0 # modulus on full rotation of 36 degrees
    return azimuth

def images_from_bag(bagfile, img_topic):
    rawlist = []
    timestamps = []
    bag = rosbag.Bag(bagfile)
    for topic, msg, t in bag.read_messages(topics=[img_topic]):
        #t_topic = msg.header.stamp.secs + 1E-9*msg.header.stamp.nsecs - t0
        t_topic = t.to_sec()
        if topic == img_topic:
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            rawlist.append(raw)
            timestamps.append(t_topic)
    if msg._type == "sensor_msgs/Image":
        info = {}
        info['type'] = msg.encoding
        info['height'] = msg.height
        info['width'] = msg.width
    elif msg._type == "sensor_msgs/CompressedImage":
        info = {}
        info['type'] = msg.format
    else:
        info = {}
        info['type'] = ""
    return rawlist, timestamps, info

def depths_from_bag(bagfile, depth_topic):
    rawlist = []
    timestamps = []
    bag = rosbag.Bag(bagfile)
    for topic, msg, t in bag.read_messages(topics=[depth_topic]):
        #t_topic = msg.header.stamp.secs + 1E-9*msg.header.stamp.nsecs - t0
        t_topic = t.to_sec()
        if topic == depth_topic:
            info = {}
            info['w'] = msg.width
            info['h'] = msg.height
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            rawlist.append(raw)
            timestamps.append(t_topic)
    return rawlist, timestamps, info


def img_msg_raw2img(raw, info):
    if 'jpeg' in info['type']:
        img = cv2.imdecode(raw, cv2.IMREAD_COLOR)
    
    elif info['type'] == 'bgr8':
        img = np.frombuffer(raw, dtype=np.uint8).reshape(info['height'], info['width'], 3)
    
    elif info['type'] == 'rgb8':
        img = np.frombuffer(raw, dtype=np.uint8).reshape(info['height'], info['width'], 3)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    
    elif info['type'] == 'rgba8':
        # img = np.frombuffer(raw, dtype=np.uint8).reshape(info['height'], info['width'], 4)[:,:,0:3] # old method
        img = np.frombuffer(raw, dtype=np.uint8).reshape(info['height'], info['width'], 4)
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        
    elif info['type'] == 'bgra8':
        img = np.frombuffer(raw, dtype=np.uint8).reshape(info['height'], info['width'], 4)
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

    else:
        img = None
    return img

def euler_to_quaternion(roll, pitch, yaw):

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

def euler_from_quaternion(quat):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    x = quat[0]
    y = quat[1]
    z = quat[2]
    w = quat[3]
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
     
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
     
    return roll_x, pitch_y, yaw_z # in radians
  
def latlon_from_bag(bagfile, gps_topic):
	latitude, longitude, east, north = [], [], [], []
	timestamps = []
	bag = rosbag.Bag(bagfile)
	init = True
	for topic, msg, t in bag.read_messages(topics=[gps_topic]):
		#t_topic = msg.header.stamp.secs + 1E-9*msg.header.stamp.nsecs - t0
		t_topic = t.to_sec()
		if topic == gps_topic:
			latitude.append(msg.latitude)
			longitude.append(msg.longitude)
			timestamps.append(t_topic)
			if init:
				init = False
				t0 = t_topic
				refLat, refLon = msg.latitude, msg.longitude
				proj_string = " ".join(("+proj=tmerc +ellps=WGS84 +lat_0=",str(refLat), "+lon_0=",str(refLon), "+k=1.0 +x_0=0 +y_0=0 +units=m +no_defs"))
				pp = pyproj.Proj(proj_string)
				east.append(0.0)
				north.append(0.0)
			else:
				x, y = pp(msg.longitude, msg.latitude)
				east.append(x)
				north.append(y)
					
	return latitude, longitude, east, north, timestamps

def heading_from_bag(bagfile, topic):
	heading = []
	timestamps = []
	bag = rosbag.Bag(bagfile)
	for topic, msg, t in bag.read_messages(topics=[topic]):
		#t_topic = msg.header.stamp.secs + 1E-9*msg.header.stamp.nsecs - t0
		t_topic = t.to_sec()
		if topic == topic:
			roll, pitch, yaw = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
			heading.append(yaw)
			timestamps.append(t_topic)
	return timestamps, heading

def speed_from_bag(bagfile, topic):
	speed = []
	timestamps = []
	bag = rosbag.Bag(bagfile)
	for topic, msg, t in bag.read_messages(topics=[topic]):
		#t_topic = msg.header.stamp.secs + 1E-9*msg.header.stamp.nsecs - t0
		t_topic = t.to_sec()
		if topic == topic:
			speed.append(msg.twist.twist.linear.x)
			timestamps.append(t_topic)
	return timestamps, speed

def sort_bag_files(bagdir, name='raw'):
    globlist = glob.glob(os.path.join(bagdir, name+'*.bag'))
    runlist = []
    for item in globlist:
        runlist.append(int(os.path.basename(item).strip(name).strip('.bag')))
    runlist.sort()
    bagfiles_sorted = []
    for item in runlist:
        bagfiles_sorted.append(os.path.join(bagdir, name+str(item)+'.bag'))
    return bagfiles_sorted

# python make_units_notbags.py --settings /media/dev/white/data/hstt/settings/make_units.json

# def parse_boolean(value):
#     value = value.lower()
#     if value in ["true", "yes", "y", "1", "t"]:
#         return True
#     elif value in ["false", "no", "n", "0", "f"]:
#         return False
#     return False

# parser = argparse.ArgumentParser(
#     description="This script accepts one filepath and two boolean values."
# )
# parser.add_argument(
#      "--settings", default="", help="Settings file path"
# )
# args = parser.parse_args()


def make_standard_unit_data(routedir, syncdir, inputs_file, gps_topic, img_topic, depth_topic, gps_filename, img_dirname, depth_dirname, extract_images=False, extract_depths=False, create_hood_blank=False):
    all_unit_dirs = glob.glob(routedir+'/unit*')
    all_unit_dirs.sort()
    if os.path.exists(syncdir):
        shutil.rmtree(syncdir)
    os.mkdir(syncdir)

    for u, unitdir in enumerate(all_unit_dirs):
        # create output unit directory from unit base name
        unit_base_name = os.path.basename(unitdir)
        output_unit_path = os.path.join(syncdir, unit_base_name)
        unit_metafile = os.path.join(unitdir, 'unit_metadata.json')
        f = open(unit_metafile)
        unit_metadata = json.load(f)
        f.close()
        extract_depths = unit_metadata['depth_map']
        os.mkdir(output_unit_path)
        shutil.copy(unit_metafile, os.path.join(output_unit_path,'unit_metadata.json')) # copy unit meta from raw unit to sync dir

        print(f"Processing unit directory: {unitdir}")
        all_unit_bag_files = sort_bag_files(unitdir, name='raw_')
        unit_latitude, unit_longitude, unit_gpstime, unit_imgtime, unit_heading = [], [], [], [], []

        if extract_images:
            imgdir = os.path.join(output_unit_path, img_dirname)
            os.mkdir(os.path.join(imgdir))
        if extract_depths:
            depthdir = os.path.join(output_unit_path, depth_dirname)
            os.mkdir(os.path.join(depthdir))

        for bg, bagfile in enumerate(all_unit_bag_files):
            # gps from bag
            latitude, longitude, east, north, timestamps_gps = latlon_from_bag(bagfile, gps_topic)
            azimuth = heading_from_path(east, north, timestamps_gps)
            unit_gpstime.extend(timestamps_gps)
            unit_latitude.extend(latitude)
            unit_longitude.extend(longitude)
            # process and write gps data to file
            # timestamps_hdg, heading = heading_from_bag(bagfile, '/imu/data')
            unit_heading.extend(azimuth)
            # timestamps_spd, speed = speed_from_bag(bagfile, '/gps/odom')
            if extract_images:
                rawlist, timestamps_imgs, info_img = images_from_bag(bagfile, img_topic)
                unit_imgtime.extend(timestamps_imgs)
            
            if extract_images:
                # imgs from bag
                print(f"Bag number: {bg+1} of {len(all_unit_bag_files)}, Bag: {os.path.basename(bagfile)}, Started extracting bag images")
                print(f"Started extracting bag images")
                for i, imgtime in enumerate(timestamps_imgs):
                    img = img_msg_raw2img(rawlist[i], info_img)
                    imgfile = os.path.join(imgdir, '{0:.9f}'.format(imgtime,)+'.png')
                    ##### TEMP Modify Image for CLRNET Blank Band 
                    if create_hood_blank:
                        blank = np.zeros(img.shape, dtype=np.uint8)
                        img_resized = cv2.resize(img, (2208, 1016), interpolation = cv2.INTER_AREA)
                        blank[0:1016,:,:] = img_resized
                        cv2.imwrite(imgfile, blank, [cv2.IMWRITE_PNG_COMPRESSION, 0])
                    else:
                        cv2.imwrite(imgfile, img, [cv2.IMWRITE_PNG_COMPRESSION, 0])
                print(f"Bag image extraction done!")

            if extract_depths:
                print(f"Started extracting bag depth maps")
                # depths from bag
                rawlist_depth, timestamps_depth, info_depth = depths_from_bag(bagfile, depth_topic)
                for i, this_time in enumerate(timestamps_imgs):
                    if this_time >= timestamps_depth[0] and this_time <= timestamps_depth[-1]:
                        min_value, min_index = get_min(np.abs(np.array(timestamps_depth)-this_time))
                        depthtime_this =  timestamps_depth[min_index]
                        if abs(this_time - depthtime_this) <= IMG_DEPTH_TIME_TOL:
                            time_valid = True
                            depthfile = os.path.join(depthdir, '{0:.9f}'.format(this_time,)+'.bin')
                            with open(depthfile, "wb") as f:
                                rawlist_depth[min_index].tofile(f)
                print(f"Depth extraction done!")

        # resample gps and create file
        # empty lists
        timestamps_sel = []
        latitude_sel = []
        longitude_sel = []
        heading_sel = []
        speed_sel = []
        # TO DO BC
        i = 0
        timestamps_gps_ = []
        while i < len(unit_imgtime):
        # for i, imgtime in enumerate(timestamps_imgs):
            imgtime = unit_imgtime[i]
            min_value, imin = get_min(np.abs(np.array(unit_gpstime)-imgtime))
            gpstime = unit_gpstime[imin]
            timestamps_gps_.append(gpstime)
            lat, lon = unit_latitude[imin], unit_longitude[imin]
            latitude_sel.append(lat)
            longitude_sel.append(lon)
            timestamps_sel.append(imgtime)
            # add speed and heading
            # min_value, imin = get_min(np.abs(np.array(timestamps_hdg)-imgtime))
            heading_sel.append(unit_heading[imin])
            # min_value, imin = get_min(np.abs(np.array(timestamps_spd)-imgtime))
            # speed_sel.append(speed[imin])
            i = i + 1
        # write data file
        header = ['Time (sec)', 'Latitude (deg)', 'Longitude (deg)', 'Heading (deg)']
        outfile = os.path.join(unitdir, gps_filename)
        with open(outfile, 'w') as out_f:
            out_f.write(('\t'.join(header)))
            out_f.write('\n')
            for i in range(0, len(timestamps_sel)):
                list_to_write = [   '{0:.9f}'.format(timestamps_sel[i],),
                                    "%.8f" % latitude_sel[i],
                                    "%.8f" % longitude_sel[i],
                                    "%.4f" % heading_sel[i]]
                                    # "%.4f" % speed_sel[i]
                out_f.write(('\t'.join(list_to_write)))
                out_f.write('\n')
        out_f.close()
        shutil.copy(outfile, os.path.join(output_unit_path, gps_filename)) # copy gps file from raw unit to sync dir
        # Copy inputs.json from config to raw unit sync dir
        inputs_filename = os.path.basename(inputs_file)
        shutil.copy(inputs_file, os.path.join(output_unit_path, inputs_filename))
        
        # print(f"Total unit gps points [s]: {'%.0f' % len(unit_gpstime)}\n")
        print(f"Unit GPS dt [ms] (avg, std): ({'%.1f' % (1000*np.mean(np.diff(unit_gpstime)))}, {'%.1f' % (1000*np.std(np.diff(unit_gpstime)))})")
        print(f"Unit IMG dt [ms] (avg, std): ({'%.1f' % (1000*np.mean(np.diff(unit_imgtime)))}, {'%.1f' % (1000*np.std(np.diff(unit_imgtime)))})")
        print(f"Total unit time [s]: {'%.1f' % (unit_imgtime[-1]-unit_imgtime[0])}")
        print(f"Total unit images [s]: {'%.0f' % len(unit_imgtime)}\n")

    print(f"Done! All units from route {routedir} have been created in {syncdir}!")

if __name__ == '__main__':   
    extract_images = True
    extract_depths = False
    create_hood_blank = False
    routedir = '/home/dev/road_audit_data/TRC_DRIVE'
    syncdir = '/home/dev/road_audit_data/sync'
    gps_topic = '/gps/fix'
    img_topic = '/zed2/zed_node/left_raw/image_raw_color/compressed'
    depth_topic= '/zed2/zed_node/depth/depth_registered'
    gps_filename= 'gps_.txt'
    img_dirname = 'images'      
    depth_dirname = 'depths'
    make_standard_unit_data(routedir, syncdir, gps_topic, img_topic, depth_topic, gps_filename, img_dirname, depth_dirname, extract_images, extract_depths, create_hood_blank)