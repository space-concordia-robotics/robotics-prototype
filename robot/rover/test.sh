#!/bin/bash

lib_name=$1
lib_version=$2
lib_url=$3
install_path=~/Arduino/libraries
curl -fsSlL  $lib_url/archive/$lib_version.tar.gz > $lib_name.tar.gz
tar -xf $lib_name.tar.gz -C $install_path
mv $install_path/${lib_name}-library-$lib_version $install_path/${lib_name}sh

echo "Installed library ${lib_name} ${lib_url} at $install_path"
