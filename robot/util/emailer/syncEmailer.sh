EMAILER_DIR="/home/odroid/emailer"

if [ ! -d $EMAILER_DIR ]; then
    mkdir $EMAILER_DIR;
fi

cp ./* $EMAILER_DIR
cd $EMAILER_DIR
npm install

