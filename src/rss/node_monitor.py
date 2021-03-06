#!/usr/bin/env python

"""
sourcd:jsk-ros-pkg/jsk_common (https://github.com/jsk-ros-pkg/jsk_common/blob/master/jsk_topic_tools/scripts/rosping_existence.py)

rosping_existence.py
check the existence of the ros nodes and publish the result to diagnostics

"""


try:
    from xmlrpc.client import ServerProxy
except ImportError:
    from xmlrpclib import ServerProxy

from rosnode import *


def _succeed(args):
    code, msg, val = args
    if code != 1:
        raise ROSNodeException("remote call failed: %s" % msg)
    return val


def ping(node_name, max_count=None, verbose=False):
    """
    Test connectivity to node by calling its XMLRPC API
    @param node_name: name of node to ping
    @type  node_name: str
    @param max_count: number of ping requests to make
    @type  max_count: int
    @param verbose: print ping information to screen
    @type  verbose: bool
    @return: True if node pinged
    @rtype: bool
    @raise ROSNodeIOException: if unable to communicate with master
    """
    master = rosgraph.Master(ID)
    node_api = get_api_uri(master, node_name)
    if not node_api:
        # print "cannot ping [%s]: unknown node" % node_name, file=sys.stderr
        return False

    timeout = 3.

    if verbose:
        print("pinging %s with a timeout of %ss" % (node_name, timeout))
    socket.setdefaulttimeout(timeout)
    node = ServerProxy(node_api)
    lastcall = 0.
    count = 0
    acc = 0.
    try:
        while True:
            try:
                count += 1
                start = time.time()
                pid = _succeed(node.getPid(ID))
                end = time.time()

                dur = (end - start) * 1000.
                acc += dur

                if verbose:
                    print("xmlrpc reply from %s\ttime=%fms" % (node_api, dur))
                    # 1s between pings
            except socket.error as e:
                # 3786: catch ValueError on unpack as socket.error is not always a tuple
                try:
                    # #3659
                    errnum, msg = e
                    if errnum == -2:  # name/service unknown
                        p = urlparse.urlparse(node_api)
                        # print("ERROR: Unknown host [%s] for node [%s]"%(p.hostname, node_name), file=sys.stderr)
                    elif errnum == errno.ECONNREFUSED:
                        # check if node url has changed
                        new_node_api = get_api_uri(master, node_name, skip_cache=True)
                        if not new_node_api:
                            # print("cannot ping [%s]: unknown node"%node_name, file=sys.stderr)
                            return False
                        if new_node_api != node_api:
                            if verbose:
                                print(
                                "node url has changed from [%s] to [%s], retrying to ping" % (node_api, new_node_api))
                            node_api = new_node_api
                            node = ServerProxy(node_api)
                            continue
                            # print("ERROR: connection refused to [%s]"%(node_api), file=sys.stderr)
                    else:
                        pass
                        # print("connection to [%s] timed out"%node_name, file=sys.stderr)
                    return False
                except ValueError:
                    print("unknown network error contacting node: %s" % (str(e)))
            if max_count and count >= max_count:
                break
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass

    if verbose and count > 1:
        print("ping average: %fms" % (acc / count))
    return True


def checkNodeExistence(nodes):
    result = {}
    have_dead = False
    for n in nodes:
        res = ping(n, max_count=1, verbose=False)
        result[n] = res
        if not res:
            have_dead = True
    if have_dead:
        return False, result
    else:
        return True, result

