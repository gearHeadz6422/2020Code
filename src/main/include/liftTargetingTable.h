class liftTargetingTable {
	public:
		double ballForwardHigh;
		double ballForwardMid;
		double ballForwardLow;

		double ballReverseHigh;
		double ballReverseMid;
		double ballReverseLow;

		double hatchHigh;
		double hatchMid;
		double hatchLow;

		liftTargetingTable(std::string desLift) {
			if (desLift == "low") { //Lift low
				ballForwardMid = 4.95;
				ballForwardLow = 1.8;

				ballReverseHigh = 0.0;
				ballReverseMid = 0.0;
				ballReverseLow = 0.0; //Done

				hatchHigh = 0.0;
				hatchMid = 0.0;
				hatchLow = 0.0;
			} else { //Lift high
				ballForwardMid = 104.3;
				ballForwardLow = 9.9;

				ballReverseHigh = 0.0;
				ballReverseMid = 0.0;
				ballReverseLow = 0.0; //Done

				hatchHigh = 0.0;
				hatchMid = 0.0;
				hatchLow = 0.0;
			}
		}
};