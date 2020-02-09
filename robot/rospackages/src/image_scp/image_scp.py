from paramiko import SSHClient
from scp import SCPClient
import argparse
from getpass import getpass


parser = argparse.ArgumentParser(description="ssh login name")
parser.add_argument(

ssh = SSHClient()
ssh.load_system_host_keys()
ssh.connect('j_klimo@login.encs.concordia.ca')

scp = SCPClient(ssh.get_transport())

scp.put('test', recursive=True, remote_path='/home/j/j_klimo/Documents/images')
