if (i.id == 0)
	{
		// Serial.print("Theta values ");
		// Serial.print(i.id);
		// Serial.print(": ");
		// Serial.println(th[0]);
		// Serial.print(", sin value: ");
		// Serial.print(sinLut[th[0]]);
		// Serial.print(", decimal: ");
		// Serial.println(sinLut[th[0]]*0.001);

		// return k1 * (g * m3 * (L1 * sinLut[theta0] * 0.001 + L2 * sinLut[theta1] * 0.001 + L3 * sinLut[theta2] * 0.001) + g * m2 * (L1 * sinLut[theta0] * 0.001 + L2 * sinLut[theta1] * 0.001) + g * L1 * m1 * sinLut[theta0] * 0.001);

		return k1 * (g * m3 * (L1 * sinLut[theta0] + L2 * sinLut[theta0 + theta1] + (L3-LCoM3) * sinLut[theta0 + theta1 + theta2]) + g * m2 * (L1 * sinLut[theta0] + (L2-(L2-LCoM2)) * sinLut[theta0 + theta1]) + g * (L1-LCoM1) * m1 * sinLut[theta0] + g * mblock * (L1 * sinLut[theta0] + Lblock * sinLut[theta0 + theta1]));
	}
	else if (i.id == 1)
	{

		// return k2 * (g * m3 * (L2 * sinLut[theta1] * 0.001 + L3 * sinLut[theta2] * 0.001) + g * L2 * m2 * sinLut[theta1] * 0.001);

		return k2 * (g * m3 * (L2 * sinLut[theta0 + theta1] + (L3-LCoM3) * sinLut[theta0 + theta1 + theta2]) + g * (L2-LCoM2) * m2 * sinLut[theta1 + theta0] + g * mblock * Lblock * sinLut[theta1 + theta0]);
	}
	else if (i.id == 2)
	{
		// Serial.print("Theta values ");
		// Serial.print(i.id);
		// Serial.print(": ");
		// Serial.print(th[0]);
		// Serial.print(',');
		// Serial.print(th[1]);
		// Serial.print(',');
		// Serial.println(th[2]);

		// Serial.print("wrap around (sum of all Angles): ");
		// Serial.println(theta1);
		// return k3 * (g * L3 * m3 * sinLut[theta2] * 0.001);

		return k3 * (g * m3 * (L3-LCoM3) * sinLut[theta2 + theta1 + theta0]);
	}
