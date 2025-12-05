CC = gcc
CFLAGS = -Wall -Wextra -O2 -std=c11 -D_USE_MATH_DEFINES -D_GNU_SOURCE
LDFLAGS = -lm

all: route_planner

route_planner: route_planner.c 
	$(CC) $(CFLAGS) -o route_planner route_planner.c -lm

clean:
	rm -f route_planner

.PHONY: all clean