# rst.bash
# ========

#####################################################################
#                                                                   #
# Directory Paths                                                   #
#                                                                   #
#####################################################################

export BUILD="${RSTPATH}/build"
export CODEBASE="${RSTPATH}/codebase"
export LOGPATH="${RSTPATH}/log"
export DOCPATH="${RSTPATH}/doc"

export WWWPATH="${DOCPATH}/www"
export URLBASE="https://superdarn.github.io/rst-ros"

export LIBPATH="${RSTPATH}/lib"
export BINPATH="${RSTPATH}/bin"
export IPATH="${RSTPATH}/include"

export USR_CODEBASE="${RSTPATH}/usr/codebase"
export USR_LIBPATH="${RSTPATH}/usr/lib"
export USR_BINPATH="${RSTPATH}/usr/bin"
export USR_IPATH="${RSTPATH}/usr/include"

export PATH="${PATH}:${BUILD}/bin:${BUILD}/script:${RSTPATH}/bin:${RSTPATH}/usr/bin:${RSTPATH}/script"
export LD_LIBRARY_PATH="${LIBPATH}:${USR_LIBPATH}:/usr/local/cdf/lib"

#####################################################################
#                                                                   #
# Makefiles                                                         #
#                                                                   #
#####################################################################

export MAKECFG=${BUILD}/make/makecfg
export MAKEBIN=${BUILD}/make/makebin
export MAKELIB=${BUILD}/make/makelib


