#!/bin/bash
#
# Sample startup script for the gsmMuxd
#
# chkconfig: 2345 94 11
# description: gsmMuxd is serial line multiplexer
#
# processname: gsmMuxd

# Source function library.
. /etc/init.d/functions

# Path to mobilegw binary, and short-form for messages
gsmMuxExe=/usr/sbin/gsmMuxd
executable=gsmMuxd
prog=gsmMuxd
RETVAL=0
OPTIONS="-p /dev/ttyUSB0 -w -r -s /dev/mux /dev/ptmx /dev/ptmx /dev/ptmx"

# Functions
start() {

        # See if it's already running.
        pid=`pidofproc $executable`
        if [ -n "$pid" ] && ps h $pid >/dev/null 2>&1; then
	    echo -n "Starting gsmMux"
	    echo_failure
	    echo
	    return
	fi
  	
        action "Starting gsmMux " $gsmMuxExe $OPTIONS
        RETVAL=$?
        touch /var/lock/subsys/gsmMuxd0
	echo          
        return $RETVAL
}
stop() {
        echo -n "Stopping gsmMux"
	killproc $gsmMuxExe
	RETVAL=$?
	rm -f /var/lock/sybsys/gsmMuxd0
	echo
}

# See how we were called.
case "$1" in
  start)
	start
	;;
  stop)
	stop
	;;
  status)
        status $gsmMuxExe
	RETVAL=$?
	;;
  restart)
	stop
	start
	;;
  condrestart)
        stop
        start
	;;
  *)
	echo $"Usage: $prog {start|stop|restart|condrestart|status}"
	exit 1
esac

exit $RETVAL
