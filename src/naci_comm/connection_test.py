"""
reference: sourceperl/th_pinger.py (https://gist.github.com/sourceperl/10288663)

"""
import subprocess
import re


def pinger(ip):
    # ping it
    args = ['/bin/ping', '-c', '3', '-W', '1', str(ip)]
    p_ping = subprocess.Popen(args,
                              shell=False,
                              stdout=subprocess.PIPE)
    # save ping stdout
    p_ping_out = p_ping.communicate()[0]

    if p_ping.wait() == 0:
        # rtt min/avg/max/mdev = 22.293/22.293/22.293/0.000 ms
        search = re.search(r'rtt min/avg/max/mdev = (.*)/(.*)/(.*)/(.*) ms',
                         p_ping_out, re.M|re.I)
        # return /max, /mdev
        return search.group(3), search.group(4)
    else:
        return "Pinger Crash."



