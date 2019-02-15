terminator -T "ServerListenerWindow" -e "./ServerListenerLocal.py" &
terminator -T "ClientSenderWindow" -e "./ClientSenderLocal.py 127.0.0.1 5000" &
