# clean
rm -f gpm2xterm
# compile
gcc -o gpm2xterm gpm2xterm.c -lgpm && chmod +x gpm2xterm
