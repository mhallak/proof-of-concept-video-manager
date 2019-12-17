alias a=alias
alias h=history
alias j=jobs
alias d=date
alias rm='rm -i'
alias ver='lsb_release -a'
#problem that libGL.so is linked to mesa and not nvidia
#export LD_LIBRARY_PATH=/usr/lib/nvidia-384:${LD_LIBRARY_PATH}
alias path='(IFS=:;ls -1d $PATH |  nl)'
#Allow core dump
ulimit -c unlimited
export PATH="/home/motek/scripts:/home/motek/scripts/python:/usr/local/bin:/usr/share:$PATH"
export EDITOR='vi'
export JAVA_HOME=$(readlink -f /usr/bin/javac | sed "s:bin/javac::")
source ~/.git-completion.bash

export GIT_MERGE_AUTOEDIT=no
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
#export GST_DEBUG=3
alias san=camera_sanity.sh
