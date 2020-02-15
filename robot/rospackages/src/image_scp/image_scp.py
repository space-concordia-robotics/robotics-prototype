from paramiko import SSHClient
from scp import SCPClient
from config import (host, user, remote_path_1, local)
ssh = SSHClient()
ssh.load_system_host_keys()
ssh.connect(hostname=host, username=user)

scp = SCPClient(ssh.get_transport())

scp.put(local, recursive=True,
        remote_path=remote_path_1)
