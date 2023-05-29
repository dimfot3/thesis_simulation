#!/bin/bash

wget --load-cookies /tmp/cookies.txt "https://docs.google.com/uc?export=download&confirm=$(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate 'https://docs.google.com/uc?export=download&id=1tx_Km4OHJxzaoKkMi6DDNt0QHsrwEFrI' -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')&id=1tx_Km4OHJxzaoKkMi6DDNt0QHsrwEFrI" -O bedroom.zip && rm -rf /tmp/cookies.txt
unzip bedroom.zip
rm bedroom.zip

wget --load-cookies /tmp/cookies.txt "https://docs.google.com/uc?export=download&confirm=$(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate 'https://docs.google.com/uc?export=download&id=1xzYyGJfddlzOgr16DMkZ6A1u_8xhAAf1' -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')&id=1xzYyGJfddlzOgr16DMkZ6A1u_8xhAAf1" -O workspaceA.zip && rm -rf /tmp/cookies.txt
unzip workspaceA.zip
rm workspaceA.zip

