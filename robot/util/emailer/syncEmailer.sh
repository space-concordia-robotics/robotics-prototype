EMAILER_DIR="/home/odroid/emailer"

if [ ! -d $EMAILER_DIR ]; then
    mkdir $EMAILER_DIR;
fi

cp ./* $EMAILER_DIR
cd $EMAILER_DIR

# install dependencies defined in package.json
npm install
