
PWD="$( cd "$( dirname "${BASH_SOURCE[0]}" )/" && pwd -P )"
LD_LIBRARY_PATH=${PWD}:${PWD}/lib:${PWD}/lib/depends:${PWD}/lib/depends/lib ${PWD}/bin/mfrmaster