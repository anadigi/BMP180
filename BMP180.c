#include <stdio.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdlib.h>
#include <math.h>
#include <getopt.h>

#define BMP180_ADDR 0x77

int bmp180_get_cal_param(int file, char *buf);
long bmp180_get_ut(int file);
long bmp180_get_up(int file, short oss);
void print_usage(FILE *stream, int exit_code);

static const char *short_options = "htps:a:";
const char *program_name;

static const struct option long_options[] = {
	{"help", 	no_argument, 		NULL, 'h'},
	{"temperature", no_argument, 		NULL, 't'},
	{"pressure", 	no_argument, 		NULL, 'p'},
	{"oss",		required_argument,	NULL, 's'},
	{"altitude",	required_argument,	NULL, 'a'},
	{NULL,		no_argument,		NULL, 0}
};

int main(int argc, char **argv) {

	int next_option;
	int oss = 0;
	int temp_opt = 0;
	int pres_opt = 0;
	double alti_opt = 0;
	program_name = argv[0];

	do {
                next_option = getopt_long(argc, argv, short_options, long_options, NULL);

                switch (next_option)
                {
                        case 'h':
                        print_usage(stdout, EXIT_SUCCESS);
                        break;

                        case 't':
                        temp_opt = 1;
                        break;

                        case 'p':
                        pres_opt = 1;
                        break;

                        case 's':
			oss = atoi(optarg);
                        printf("OSS set to %d\n", oss);
                        break;

			case 'a':
			alti_opt = atof(optarg);
                        break;

                        case '?':
                        print_usage(stdout, EXIT_SUCCESS);
                        break;

                        case -1:
                        break;

                        default:
                        print_usage(stdout, EXIT_SUCCESS);
                        break;
                }
        } while (next_option != -1);

	//CALIBRATION PARAMETERS
	short AC1, AC2, AC3, B1, B2, MB, MC, MD;
	unsigned short AC4, AC5, AC6;

	if(argc > 1) {
		printf("****** BMP180 PRESSURE SENSOR TEST PROGRAM *******\n");
	}
	else {
		printf("No arguments given.\n");
		print_usage(stdout, EXIT_FAILURE);
	}

	int file;
	char *filename = "/dev/i2c-1";
	char buf[22];

	file = open(filename, O_RDWR);
	if(file < 0) {
		perror("BMP180:Fail opening i2c device file");
		close(file);
		exit(EXIT_FAILURE);
	}

	if(ioctl(file, I2C_SLAVE, BMP180_ADDR) < 0) {
		perror("BMP180:Fail selecting i2c device");
		close(file);
		exit(EXIT_SUCCESS);
	}

	bmp180_get_cal_param(file, buf);

	//SET CALIBRATION PARAMETERS READ FROM THE SENSOR
	AC1 = buf[0] << 8 | buf[1];
	AC2 = buf[2] << 8 | buf[3];
	AC3 = buf[4] << 8 | buf[5];
	AC4 = buf[6] << 8 | buf[7];
	AC5 = buf[8] << 8 | buf[9];
	AC6 = buf[10] << 8 | buf[11];
	B1 = buf[12] << 8 | buf[13];
	B2 = buf[14] << 8 | buf[15];
	MB = buf[16] << 8 | buf[17];
	MC = buf[18] << 8 | buf[19];
	MD = buf[20] << 8 | buf[21];

	long UT, X1, X2, B5, T;

	UT = bmp180_get_ut(file);
	X1 = (UT - AC6) * AC5 / 32768;
	X2 = MC * 2048 /(X1 + MD);
	B5 = X1 + X2;
	T = (B5 + 8) / 16;
	if(temp_opt == 1) {
		printf("Temperature: %.2f C\n", ((double)T)/10);
	}

	long UP;
	long B6, X3, B3;
	unsigned long B4, B7;
	long PRES;
	UP = bmp180_get_up(file, oss);
	B6 = B5 - 4000;
	X1 = (B2 * (B6 * B6 / 4096)) / 2048;
	X2 = AC2 * B6 / 2048;
	X3 = X1 + X2;
	B3 = (((AC1 * 4 + X3) << oss) + 2) / 4;
	X1 = AC3 * B6 / 8192;
	X2 = (B1 * (B6 * B6 / 4096)) / 65536;
	X3 = ((X1 + X2) + 2) / 4;
	B4 = AC4 * (unsigned long)(X3 + 32768) / 32768;
	B7 = ((unsigned long)UP - B3) * (50000 >> oss);
	if(B7 < 0x80000000) {
		PRES = (B7 * 2) / B4;
	}
	else {
		PRES = (B7 / B4) * 2;
	}
	X1 = (PRES / 256) * (PRES / 256);

	X1 = (X1 * 3038) / 65536;
	X2 = (-7357 * PRES) / 65536;
	PRES = PRES + (X1 + X2 + 3791) / 16;

	if(pres_opt == 1) {
		printf("Pressure %.2f in hPa\n", (double)PRES / 100);
	}

	double pressure = (double)PRES / 100;
	double temp = 1 - (alti_opt / 44330);
	double pressure_sea = pressure / pow(temp, 5.255);
	if(alti_opt != 0) {
		printf("Pressure at sea level: %.2f hPa\n", pressure_sea);
	}

	close(file);
	return (EXIT_SUCCESS);
}

int bmp180_get_cal_param(int file, char *buf) {

	int response;
	response = i2c_smbus_read_i2c_block_data(file, 0xAA, 22, buf);
	if(response != 22) {
		perror("BMP180:Calibration data read failed.");
		close(file);
		exit(EXIT_FAILURE);
	}

	return(EXIT_SUCCESS);
}

long bmp180_get_ut(int file) {

	long response;

	if(i2c_smbus_write_byte_data(file, 0xF4, 0x2E) != 0) {
		perror("BMP180:Register write failed.");
		close(file);
		exit(EXIT_FAILURE);
	}
	usleep(5000);
	response = i2c_smbus_read_word_data(file, 0xF6);
	response = ((response & 0x00FF) << 8) | ((response & 0xFF00) >>8);

	return response;
}

long bmp180_get_up(int file, short oss) {

	long up;
	char buffer[3];
	int response;
	char command;
	command = (0x34 + (oss << 6));

	if(i2c_smbus_write_byte_data(file, 0xF4, command) != 0) {
		perror("BMP180:Presure read initialization write failed");
		close(file);
		exit(EXIT_FAILURE);
	}
	usleep(30000);
	response = i2c_smbus_read_i2c_block_data(file, 0xF6, 3, buffer);
	if(response != 3) {
                perror("BMP180:Pressure raw data read failed.");
                close(file);
                exit(EXIT_FAILURE);
        }

	up = ((buffer[0] << 16) | (buffer[1] << 8) | buffer[2]) >> (8 - oss);

	return up;
}

void print_usage(FILE *stream, int exit_code)
{
  fprintf(stream, "Usage:  %s [options] [ -s 3 | -a 15 ... ]\n", program_name);
  fprintf(stream,
        "  -h  --help                   Display this usage information.\n"
        "  -t  --temperature            Display temperature value.\n"
        "  -p  --pressure               Display absolute air pressure value.\n"
        "  -s  --oss                    Set oversampling ratio. Valid values [0,1,2,3].\n"
        "  -a  --altitude               Set sensor altitude from the sea level in meters to display pressure at sea level.\n");
  exit (exit_code);
}
