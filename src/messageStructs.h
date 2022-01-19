//Bland message to just get the first char
typedef struct message_t {
	char type;
	char padding[3];
	unsigned char theMessage[240];
};

typedef union BytePacket_t{
	message_t message;
	unsigned char BytePacket[sizeof(message_t)];
};

//Bland message to just get the first char
typedef struct debug_t {
	char type;
	char padding[7];
	unsigned char string[100];
};

typedef union DebugPacket_t{
	debug_t message;
	unsigned char BytePacket[sizeof(debug_t)];
};

//Joint Goal message
typedef struct poseGoal_t {
	char type;
	char padding[7];
	double j0;
	double j1;
	double j2;
	double j3;
	double j4;
};
typedef union posePacket_t{
	poseGoal_t message;
	unsigned char BytePacket[sizeof(poseGoal_t)];
};
//PID Message
typedef struct PID_t{
    char type;
    char padding[7];
    double j0F[3];
    double j1F[3];
    double j2F[3];
    double j3F[3];
    double j4F[3];
    double j0B[3];
    double j1B[3];
    double j2B[3];
    double j3B[3];
    double j4B[3];
};

typedef union PID_Packet {
    PID_t message;
    unsigned char BytePacket[240];
};

//Magnet Message
typedef struct magnetState_t{
    char type;
    char padding[7];
    int magnet1;
    int magnet2;
};

typedef union magnetPacket_t{
    magnetState_t message;
    unsigned char BytePacket[sizeof(magnetState_t)];
};

