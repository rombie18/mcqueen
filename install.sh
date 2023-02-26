#!/bin/bash
chmod +x mcqueen-service.sh

cp mcqueen.service /etc/systemd/system/mcqueen.service
systemctl daemon-reload
systemctl enable mcqueen.service
systemctl start mcqueen.service