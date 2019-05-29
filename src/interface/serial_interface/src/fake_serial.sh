echo "Run manually with parameters depending on the first command"
socat -d -d pty,raw,echo=0 pty,raw,echo=0 &
cat < /dev/pts/x
