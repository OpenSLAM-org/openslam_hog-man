VERBOSE=0

ifdef DEBUG 
	CXXFLAGS+= -g -O0 -Wall -frtti 
	CFLAGS+= -g -O0 -Wall 
else
	CXXFLAGS+= -O3 -Wall -frtti -mmmx -msse3  
	CFLAGS+= -O3 -Wall -mmmx -msse3
endif
