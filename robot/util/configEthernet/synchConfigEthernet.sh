ETHERNET_CONFIG_DIR="/home/$USER/configEthernet"

if [ ! -d $ETHERNET_CONFIG_DIR ]; then
    mkdir $ETHERNET_CONFIG_DIR;
fi

cp ./* $ETHERNET_CONFIG_DIR

cd $ETHERNET_CONFIG_DIR
echo 0 > status_done
