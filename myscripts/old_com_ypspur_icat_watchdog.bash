#!/bin/bash
# Usage: ./icart_watchdog.sh start|stop|restart|check

SERVICE_PREFIX="/icart"

if [ $# -ne 1 ]; then
    echo "Usage: $0 start|stop|restart|check"
    exit 1
fi

case "$1" in
    start)
        SERVICE_NAME="${SERVICE_PREFIX}_start"
        ;;
    stop)
        SERVICE_NAME="${SERVICE_PREFIX}_stop"
        ;;
    restart)
        SERVICE_NAME="${SERVICE_PREFIX}_restart"
        ;;
    check)
        SERVICE_NAME="${SERVICE_PREFIX}_status"
        ;;
    *)
        echo "Invalid argument: $1"
        echo "Usage: $0 start|stop|restart|check"
        exit 1
        ;;
esac

echo "Calling service $SERVICE_NAME ..."
ros2 service call $SERVICE_NAME std_srvs/srv/Trigger "{}"
