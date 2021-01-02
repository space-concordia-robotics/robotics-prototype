EMAILER_DIR="/home/$USER/emailer"

if [ ! -d $EMAILER_DIR ]; then
    mkdir $EMAILER_DIR;
fi

cp ./* $EMAILER_DIR
cd $EMAILER_DIR

cp runEmailer.sh /usr/bin/runEmailer.sh
# install dependencies defined in package.json
npm install
