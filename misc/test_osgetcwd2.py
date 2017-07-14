#! /usr/bin/env python
# Copyright 2017 by University of Cincinnati
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/rss_git_lite

# this doesn't need import above for gC because of test_osgetcwd .runthis2() call
def trythis():
    import test_osgetcwd as t_os
    t_os.run_this()
    print("")
    t_os.run_this2()

    #import getConnectionIPaddress as gC # this doesn't work if run from subdir ./misc2/test_osgetcwd3.py
    from rss_git_lite.common import getConnectionIPaddress as gC # this works if imported and called from test_osgetcwd3.py because common sys.path used for entire python instance and test_osgetcwd3.py already modified the sys.path to find this!
    print("gC.dns_lookup_or_exit('google.com') = %r " % gC.dns_lookup_or_exit('google.com'))

# this doesn't need import above for gC because of test_osgetcwd .runthis2() call
if __name__ is '__main__':
    import test_osgetcwd as t_os
    t_os.run_this()
    print("")
    t_os.run_this2()

    import getConnectionIPaddress as gC
    print("gC.dns_lookup_or_exit('google.com') = %r " % gC.dns_lookup_or_exit('google.com'))



# --eof--
