ETHERNET_CONFIG_DIR="/home/odroid/configEthernet"

if [ ! -d $ETHERNET_CONFIG_DIR ]; then
    mkdir $ETHERNET_CONFIG_DIR;
fi

cp ./* $ETHERNET_CONFIG_DIR
