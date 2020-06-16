#!/bin/bash

lib_name=$1
lib_version=$2
lib_url=$3
install_path=~/Arduino/libraries
curl -fsSlL  $lib_url/archive/$lib_version.tar.gz > $lib_name.tar.gz
tar xf $lib_name.tar.gz --transform 's,^[^/]\+\($\|/\),'"$lib_name"'\1,' -C $install_path
rm -rf $lib_name.tar.gz

echo "-- Installed library $lib_name $lib_version at $install_path"
