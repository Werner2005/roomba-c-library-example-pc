#include <roomba.h>
#include <roomba_print.h>
#include <roomba_command.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <termios.h>

#include <signal.h>
#include <pthread.h>

#define BAUT B115200
/*#define BAUT B57600*/

#define TEST
/*#define BAUDCHANGE*/

/**
 * File Diskriptor
 */
static int dev;
/*
 * Lese Thread
 */
static pthread_t readpThread;
/**
 * Bestiimt ob die Anwendung leuft oder nicht
 */
static volatile bool isRuning = true;

/**
 * Write Handler fuer Roomba Libaray
 * @param c Zeichen das gesendet worden soll
 */
int writeHandler(unsigned char c){
	int ret;
	ret = write(dev, &c, 1);
	/*printf("write: %c = %x\n", c, c);*/
	if(ret < 0){
		perror("write");
	}
	return ret;
}

/**
 * Read Thread
 * leitet alle Zeichen die gelsenen wird umgehen zur Roomba Libary um
 * @param msg Wird nicht genutzt
 * @return Wird nicht genutzt
 */
void* readThread(void* msg) {
	int ret;
	unsigned char c;
	printf("Start Listening\n");
	while(isRuning){
		ret = read(dev, &c, 1);
		if(ret < 0){
			perror("read");
		}else{
			/*printf("read: %c = %x\n", c, c);*/
			roomba_read(c);
		}
	}
	printf("End Listening\n");
	return NULL;
}
/**
 * RS232 Init
 * @param device Device das aufgerufen werden soll
 */
void rs232init(char* device){
	/*
	 * Bassiert auf roombacmd roombalib.c
	 */
	struct termios toptions;
	int ret;
	/*
	 * Oeffnen
	 */
	dev = open(device, O_RDWR);
	if(dev == -1){
		perror("open");
		exit(EXIT_FAILURE);
	}
	/*
	 * Hohle anktuelle Einstellungen vom Gereat
	 */
	ret = tcgetattr(dev, &toptions);
	if (ret != 0) {
		perror("tcgetattr");
		ret = close(dev);
		if(ret != 0){
			perror("close");
		}
		exit(EXIT_FAILURE);
	}
	/*
	 * Setze Input Speed
	 */
	ret = cfsetispeed(&toptions, BAUT);
	if(ret != 0){
		perror("cfsetispeed");
		ret = close(dev);
		if(ret != 0){
			perror("close");
		}
		exit(EXIT_FAILURE);
	}
	/*
	 * Setze Output Speed
	 */
	ret = cfsetospeed(&toptions, BAUT);
	if(ret != 0){
		perror("cfseoispeed");
		ret = close(dev);
		if(ret != 0){
			perror("close");
		}
		exit(EXIT_FAILURE);
	}

	/*
	 * Setze 8 Bit 
	 * Keine Parität
	 * 1 Stopbit
	 */
	toptions.c_cflag &= ~PARENB;
	toptions.c_cflag &= ~CSTOPB;
	toptions.c_cflag &= ~CSIZE;
	toptions.c_cflag |= CS8;
	/*
	 * Keine Flow Contol
	 */
	toptions.c_cflag &= ~CRTSCTS;

	/*
	 * Aktiviere Read 
	 * Ignoriere Modem Steuer Zeichen
	 */
	toptions.c_cflag    |= CREAD | CLOCAL;
	/*
	 * Deaktive Software Flow Contoll
	 */
	toptions.c_iflag    &= ~(IXON | IXOFF | IXANY);
	/*
	 * Deaktiviere Zeilenweiser Input(ICANON)
	 * Deaktiviere Echo Mode
	 * Deaktiviere Echo Mode Backspace
	 * Deaktiviere Singal Erkkenung
	 */
	toptions.c_lflag    &= ~(ICANON | ECHO | ECHOE | ISIG);
	/*
	 * Deaktive OPOST
	 */
	toptions.c_oflag    &= ~OPOST;
	
	/*
	 * Minimale Zeichen anzahl fuer Read
	 * 1 Byte da es Sensor Daten gibt die nur 1 Byte gros sind
	 */
	toptions.c_cc[VMIN]  = 1;
	/*
	 * Kein Timeout
	 * Roomba Protokoll ist Asyncron
	 */
	toptions.c_cc[VTIME] = 0;
	
	/*
	 * Schreibe Einstellungen nach tty
	 */
	ret = tcsetattr(dev, TCSANOW, &toptions);
	ret = tcsetattr(dev, TCSANOW, &toptions);
	if(ret != 0){
		perror("tcsetattr");
		ret = close(dev);
		if(ret != 0){
			perror("close");
		}
		exit(EXIT_FAILURE);
	}
}
/**
 * Signal Handler 
 * Wird benoetigt um read mit hilfe von SIGUSR1 zu unterbrechen
 * @param i SigID
 */
void sigHandler(int i){
	printf("Signal gefangen\n");
}

/**
 * Inizaliseren Reader
 */
void readinit(){
	int ret;
	pthread_attr_t attr; /* Eigenschaften von PThreads*/
	struct sigaction sigac;

	/*
	 * Signal Handler setzten
	 */
	memset(&sigac, 0, sizeof(struct sigaction));

	sigac.sa_handler = sigHandler;

	ret = sigaction(SIGUSR1, &sigac, NULL);
	if(ret < 0){
		perror("sigaction");
		ret = close(dev);
		if(ret != 0){
			perror("close");
		}
		exit(EXIT_FAILURE);
	}

	/*
	 * Pthread erstellen
	 */
	pthread_attr_init(&attr);
	ret = pthread_create(&readpThread, &attr, readThread, NULL);
	if (ret != 0) {
		perror("pthread_create");
		pthread_attr_destroy(&attr);
		ret = close(dev);
		if(ret != 0){
			perror("close");
		}
		exit (EXIT_FAILURE);
	}
	pthread_attr_destroy(&attr);
}
void cleanUp(){
	void* tmp;
	int ret;
	ret = pthread_join(readpThread, &tmp);
	if(ret != 0){
		perror("pthread_join");
		exit(EXIT_FAILURE);
	}

	ret = close(dev);
	if(ret != 0){
		perror("close");
		exit(EXIT_FAILURE);
	}
}
int8_t read_sim(uint8_t c){
	int tmp;
	printf("Schreibe: %x\n", c);
	tmp = write(dev, &c, 1);
	if(tmp < 0){
		perror("write");
	}
	sleep(1);
	return tmp;
}

void initRoomba(){
	int ret;
	roombaCommand_t c;
	roombaSensorData_t s;

	c.command = ROOMBA_START;
	c.size = 0;

	ret = roomba_sendCommand(&c);

	if(ret < 0){
		fprintf(stderr, "Kann Start Kommando nicht senden ERROR: %d\n", ret);
		exit(EXIT_FAILURE);
	}

	c.command = ROOMBA_SAFE;
	c.size = 0;

	ret = roomba_sendCommand(&c);
	if(ret < 0){
		fprintf(stderr, "Kann Safe Kommando nicht senden ERROR: %d\n", ret);
		exit(EXIT_FAILURE);
	}

	c.command = ROOMBA_SENSORS;
	c.size = 1;
	c.data[0] = ROOMBA_SENSOR_OI_MODE;


	ret = roomba_sendCommand(&c);
	if(ret < 0){
		fprintf(stderr, "Kann Sensor Kommando nicht senden ERROR: %d\n", ret);
		exit(EXIT_FAILURE);
	}

	ret = roomba_getSensorData(&s);
	if(ret < 0){
		fprintf(stderr, "Kann keine Senor Daten erreichen nicht senden ERROR: %d\n", ret);
		exit(EXIT_FAILURE);
	}

	if(s.data != ROOMBA_SENSOR_OI_MODE_SAFE){
		fprintf(stderr, "Roomba hat nicht in den Safe Mode gewechselt OI ID: %x\n", s.data);
	}
	printf("Init erflogreich OI Mode: %x\n", s.data);
}
void baud(){
	int ret;
	roombaCommand_t c;
	c.command = ROOMBA_BAUD;
	c.size = 1; 
	c.data[0] = 10;

	ret = roomba_sendCommand(&c);
	if(ret < 0){
		fprintf(stderr, "Kann Drive Kommando nicht senden ERROR: %d\n", ret);
		exit(EXIT_FAILURE);
	}
	
}
static int8_t getGroupStart(int8_t c){
	switch(c){
		case ROOMBA_SENSOR_GROUP_0:
			return ROOMBA_SENSOR_GROUP_0_START;
		case ROOMBA_SENSOR_GROUP_1:
			return ROOMBA_SENSOR_GROUP_1_START;
		case ROOMBA_SENSOR_GROUP_2:
			return ROOMBA_SENSOR_GROUP_2_START;
		case ROOMBA_SENSOR_GROUP_3:
			return ROOMBA_SENSOR_GROUP_3_START;
		case ROOMBA_SENSOR_GROUP_4:
			return ROOMBA_SENSOR_GROUP_4_START;
		case ROOMBA_SENSOR_GROUP_5:
			return ROOMBA_SENSOR_GROUP_5_START;
		case ROOMBA_SENSOR_GROUP_6:
			return ROOMBA_SENSOR_GROUP_6_START;
		case ROOMBA_SENSOR_GROUP_100:
			return ROOMBA_SENSOR_GROUP_100_START;
		case ROOMBA_SENSOR_GROUP_101:
			return ROOMBA_SENSOR_GROUP_101_START;
		case ROOMBA_SENSOR_GROUP_106:
			return ROOMBA_SENSOR_GROUP_106_START;
		case ROOMBA_SENSOR_GROUP_107:
			return ROOMBA_SENSOR_GROUP_107_START;
	}
	return -1;
}
static int8_t getGroupEnd(int8_t c){
	switch(c){
		case ROOMBA_SENSOR_GROUP_0:
			return ROOMBA_SENSOR_GROUP_0_STOP;
		case ROOMBA_SENSOR_GROUP_1:
			return ROOMBA_SENSOR_GROUP_1_STOP;
		case ROOMBA_SENSOR_GROUP_2:
			return ROOMBA_SENSOR_GROUP_2_STOP;
		case ROOMBA_SENSOR_GROUP_3:
			return ROOMBA_SENSOR_GROUP_3_STOP;
		case ROOMBA_SENSOR_GROUP_4:
			return ROOMBA_SENSOR_GROUP_4_STOP;
		case ROOMBA_SENSOR_GROUP_5:
			return ROOMBA_SENSOR_GROUP_5_STOP;
		case ROOMBA_SENSOR_GROUP_6:
			return ROOMBA_SENSOR_GROUP_6_STOP;
		case ROOMBA_SENSOR_GROUP_100:
			return ROOMBA_SENSOR_GROUP_100_STOP;
		case ROOMBA_SENSOR_GROUP_101:
			return ROOMBA_SENSOR_GROUP_101_STOP;
		case ROOMBA_SENSOR_GROUP_106:
			return ROOMBA_SENSOR_GROUP_106_STOP;
		case ROOMBA_SENSOR_GROUP_107:
			return ROOMBA_SENSOR_GROUP_107_STOP;
	}
	return -1;
}

void readAllSensorData(uint8_t group){
	roombaCommand_t c;
	roombaSensorData_t s;
	int i;
	int ret;
	c.command = ROOMBA_SENSORS;
	c.size = 1;
	c.data[0] = group;

	ret = roomba_sendCommand(&c);
	if(ret < 0){
		fprintf(stderr, "Kann Drive Kommando nicht senden ERROR: %d\n", ret);
		exit(EXIT_FAILURE);
	}
	sleep(1);

	for(i = getGroupStart(group); i <= getGroupEnd(group); i++){
		if(roomba_hasSensorData()){
			ret = roomba_getSensorData(&s);
			if(ret < 0){
				fprintf(stderr, "Kann keine Senor Daten erreichen nicht senden ERROR: %d\n", ret);
				exit(EXIT_FAILURE);
			}
			roomba_printSensorData(&s);
		}else{
			printf("!!!!!!1SensorID: %d konnte nicht abgerufen Werden\n", i);
		}
		
	}
	roomba_reset();
}

void readSensorID(uint8_t sensorID){
	roombaCommand_t c;
	roombaSensorData_t s;
	int ret;
	c.command = ROOMBA_SENSORS;
	c.size = 1;
	c.data[0] = sensorID;

	ret = roomba_sendCommand(&c);
	if(ret < 0){
		fprintf(stderr, "Kann Drive Kommando nicht senden ERROR: %d\n", ret);
		exit(EXIT_FAILURE);
	}
	sleep(1);
	if(roomba_hasSensorData()){
		ret = roomba_getSensorData(&s);
		if(ret < 0){
			fprintf(stderr, "Kann keine Senor Daten erreichen nicht senden ERROR: %d\n", ret);
			exit(EXIT_FAILURE);
		}
		roomba_printSensorData(&s);
	}else{
		printf("!!!!!!1SensorID: %d konnte nicht abgerufen Werden\n", sensorID);
		roomba_reset();
	}
}

int main(int argc, char** argv){
	/*int i;*/
	if(argc <= 1){
		printf("Syntax: %s <Serial Dev>\n", argv[0]);
		return EXIT_FAILURE;
	}
	rs232init(argv[1]);
	roomba_init(&writeHandler);
	readinit();
	sleep(1);
	initRoomba();
	sleep(1);

	
	roomba_drive(20, 0);
	sleep(3);
	roomba_drive(-20, 0);
	sleep(3);
	roomba_drive(0,0);

	roomba_drive(20, 1);
	sleep(3);
	roomba_drive(20, -1);
	sleep(3);
	roomba_drive(0,0);
	
	#ifdef TEST
	printf("-------------------------------------------------\n");
	printf("Group: 0\n");
	readAllSensorData(ROOMBA_SENSOR_GROUP_0);
	printf("-------------------------------------------------\n");
	printf("Group: 1\n");
	readAllSensorData(ROOMBA_SENSOR_GROUP_1);
	printf("-------------------------------------------------\n");
	printf("Group: 2\n");
	readAllSensorData(ROOMBA_SENSOR_GROUP_2);
	printf("-------------------------------------------------\n");
	printf("Group: 3\n");
	readAllSensorData(ROOMBA_SENSOR_GROUP_3);
	printf("-------------------------------------------------\n");
	printf("Group: 4\n");
	readAllSensorData(ROOMBA_SENSOR_GROUP_4);
	printf("-------------------------------------------------\n");
	printf("Group: 5\n");
	readAllSensorData(ROOMBA_SENSOR_GROUP_5);
	printf("-------------------------------------------------\n");
	printf("Group: 6\n");
	readAllSensorData(ROOMBA_SENSOR_GROUP_6);
	printf("-------------------------------------------------\n");
	printf("Group: 100\n");
	readAllSensorData(ROOMBA_SENSOR_GROUP_100);
	printf("-------------------------------------------------\n");
	printf("Group: 101\n");
	readAllSensorData(ROOMBA_SENSOR_GROUP_101);
	printf("-------------------------------------------------\n");
	printf("Group: 106\n");
	readAllSensorData(ROOMBA_SENSOR_GROUP_106);
	printf("-------------------------------------------------\n");
	printf("Group: 107\n");
	readAllSensorData(ROOMBA_SENSOR_GROUP_107);
	#endif

	/*for(i = 7; i < 100; i++){
		printf("-------------------------------------------------\n");
		readSensorID(i);
	}*/

	
	sleep(2);
	isRuning = false;


	/*
	 * Interrup read
	 */
	printf("Exit... Interrupt read\n");
	pthread_kill(readpThread, SIGUSR1);

	cleanUp();

	return 0;
}
