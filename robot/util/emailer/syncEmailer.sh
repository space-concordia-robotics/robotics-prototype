EMAILER_DIR="/home/odroid/emailer"

if [ ! -d $EMAILER_DIR ]; then
    mkdir $EMAILER_DIR;
else
    echo "There already exists an emailer directory in /home/odroid"
    echo "Please rename or delete the folder for this script to proceed"
    exit 1
fi

cp ./* $EMAILER_DIR
cd $EMAILER_DIR
npm install

