#! /bin/sh

## this is the local folder where the current units reside
sync_dir=/tmp/collect

## this is the s3 bucket and folder where the collect will be uploaded to
s3location=s3://tac-image-staging/
timestamp=$(date +"%Y-%m-%d-%H-%M")

## this will sync all files and folders in the local folder to the s3 bucket
aws s3 sync $sync_dir "$s3location$timestamp/"
