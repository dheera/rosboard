#!/bin/bash

if [ ! -f '/opt/ros/kinetic/share/rosbridge_suite/package.xml' ]; then
    echo "installing ros-kinetic-rosbridge-suite ..."
    sudo apt-get install -y ros-kinetic-rosbridge-suite
    echo "installing ros-kinetic-web-video-server ..."
    sudo apt-get install -y ros-kinetic-web-video-server
    echo 'Please be sure to include the following in your launch file:'
    echo
    echo '  <include file="$(find rosbridge_server)/launch/rosbridge_websocket.launch" />'
    echo '  <node name="web_video_server" pkg="web_video_server" type="web_video_server"></node>'
    echo
fi

if [ ! -f '/etc/nginx/sites-available/rosboard.nginx-site' ]; then
    echo "installing python dependencies ..."
    sudo pip install tornado pymongo
    sudo pip3 install tornado pymongo
    echo "installing nginx ..."
    sudo apt-get install -y nginx
    echo "adding nginx site for rosboard ..."
    sudo cp config/rosboard.nginx-site /etc/nginx/sites-available/
    echo "adding symlink from sites-available to sites-enabled ..."
    sudo ln -s /etc/nginx/sites-available/rosboard.nginx-site /etc/nginx/sites-enabled/rosboard.nginx-site
    echo "restarting nginx ..."
    sudo service nginx restart
fi

echo "updating site ..."
sudo mkdir -p /var/rosboard/html/
sudo cp -rv html/* /var/rosboard/html/
sudo chown -R 644 /var/rosboard/html/
