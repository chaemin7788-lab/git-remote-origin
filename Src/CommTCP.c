

typedef enum {
  idle,
  commandReceived,
  commandExecuted
} comState_t;

comState_t comState;

void CommTcp()
{
	switch (comState) {
	case idle:
	{
		// 코일 비트로 수신을 확인한다.
		if(getNewCommand()) // If new command arrived
		{

			comState = commandReceived;

		}
			/*
		setMovementCompleted(0); // Slave indicate that the movement is not completed
		targetDeg = getTargetPos_Deg();
		fTargetPos = ((float)(targetDeg) * (float)(M_PI)) / 180.0f;
		movementDuration_s = getMovementDuration_s();
		if (movementDuration_s > 0)
		{
		MC_ProgramPositionCommandMotor1(fTargetPos, movementDuration_s); // Send command

		}
		else
		{
		comState = commandExecuted; // Abort
		}

		}
		   */
	}
	break;

	case commandReceived:
	{
		/*
		if (isMovementComplete()) // If the movement has been completed
		{
			setMovementCompleted(1); // Slave indicate that the movement is completed
			comState = commandExecuted;
		}
		*/
	}
	break;

	case commandExecuted:
	{
		/*
		{
			clearCommand();
			comState = idle;
		}
		*/
	break;

	}
	}
}
