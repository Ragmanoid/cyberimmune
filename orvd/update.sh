#!/bin/bash
rm -r /var/www/orvd
cp -r ./ /var/www/orvd
chmod -R 777 /var/www/orvd
systemctl restart apache2