#include <stdlib.h>
#include <stdio.h>

void usage(char *argv0)
{
	printf("arkex usage:\n    %s <arg1> <arg2> <TODO>\n", argv0);
}

int main(int argc, char **argv)
{
	if (argc != 2) {
		usage(argv[0]);
	}

	return 0;
}

