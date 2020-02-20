from paramiko import SSHClient
from scp import SCPClient
#from congif import (host, user, remote_path_1, local)

class remote_connection:
    def __init__(self, host, user, remote_path, file_name):
        self.host = host
        self.user = user
        self.remote_path = remote_path
        self.file_name = file_name

    def connect(self):
        self.client = SSHClient()
        self.client.load_system_host_keys()
        self.client.connect(hostname=self.host, username=self.user)
        self.scp = SCPClient(self.client.get_transport())

    def disconnect(self):
        self.client.close()
        self.scp.close()




#ssh = SSHClient()
#ssh.load_system_host_keys()
#ssh.connect(hostname="login.encs.concordia.ca", username='user')

#scp = SCPClient(ssh.get_transport())

#scp.put('/test2.png', recursive=True,
 #       remote_path='remotepath    e)
