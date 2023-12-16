#include <stdio.h>
#include <math.h>

/*Acc roll and phi callculation

	roll (phi) , and pitch (theta)
	
	aX = g * sin(theta)
	aY = -g * cos(theta) * sin(phi)
	aZ = -g * cos(theta) * cos(phi) 
	
	RADTODEG = 180 / pi

	***acc_to_deg formula 1***

	phi_deg = atanf(aY / aZ) * RADTODEG
	theta_deg = asinf(aX / g) * RADTODEG
	
	***acc_to_deg formula 2***
	
	phi_deg = atanf(aY / (sqrt(aX * aX + aZ * aZ))) * RADTODEG;
	theta_deg = atanf(aX / (sqrt(aY * aY + aZ * aZ))) * RADTODEG;
*/

/*Low pass filter

	y[n]=(1 - alpha) * y[n-1] + alpha * x[n]
	
	filtered_data = (1 - alpha) * filtered_data(n-1) + alpha * input_data;

*/

const float pi = 3.141592;
const float rtd = 180 / 3.141592;
const float dtr = 3.141592 / 180;
const float g = 9.8;
const float alpha = 0.2;

int i = 0;

void test_acc_values();

void test_deg_values();

void callculation_deg(float aX, float aY, float aZ, float *phi, float *theta);

void callculation_acc(float p, float t, float *aX, float *aY, float *aZ);

void input_create_accelYZ_to_phi();

void input_create_accelXZ_to_theta();

void lpf(float *aX, float *aY, float *aZ);

float aX = 0.0, aY = 0.0, aZ = 0.0, phi = 0.0, theta = 0.0;
float filtered_data[3] = {0};

float X[361]={0},Y[361]={0},Z[361]={0};

int main()
{
	//test_acc_values();
	//test_deg_values();
	
	return 0;
}

void test_acc_values()
{
	//input_create_accelYZ_to_phi();
	//or
	//input_create_accelXZ_to_theta()
	lpf(&X[i], &Y[i], &Z[i]);
		for(i = 0; i<=360; i++)
	{	
	
		callculation_deg(X[i], Y[i], Z[i], &phi, &theta);
	}
}

void test_deg_values()
{
	callculation_acc(phi, theta, &aX, &aY, &aZ);
}

void input_create_accelYZ_to_phi()
{
	//input data for look input_accelYZ_to_phi.txt and paste here.
}

void input_create_accelXZ_to_theta()
{
	//input data for look input_accelXZ_to_theta.txt and paste here.
}
void callculation_deg(float aX, float aY, float aZ, float *phi, float *theta)
{
	*phi = 0.0;
	*theta = 0.0;
	
	*phi = atanf(aY / (sqrt(aX*aX + aZ*aZ))) * rtd;
	*theta = atanf(aX / (sqrt(aY*aY + aZ*aZ))) * rtd;
	
	printf("p = %.4f, t = %.4f , 		X = %f, 	Y = %f, 	Z = %f\n",*phi,*theta,aX,aY,aZ);
	
}

void callculation_acc(float phi, float theta, float *aX, float *aY, float *aZ)
{
	int i=0;
	
	for(i = 0; i<=360; i++)
	{	
		phi = 0.0;
		theta = 0.0;
		theta += i;

		*aX = sin(theta*dtr);
		*aY = -cos(theta*dtr)*sin(phi*dtr);
		*aZ = -cos(theta*dtr)*cos(phi*dtr);
		
		//printf("p = %f, t = %f , 		X = %f, 	Y = %f, 	Z = %f\n",phi,theta,*aX,*aY,*aZ);
		
		/* Create output accX, accY, accZ arrays */
		/*
			printf("X[%d] = %f;\n",i,*aX);
			printf("Y[%d] = %f;\n",i,*aY);	
			printf("Z[%d] = %f;\n",i,*aZ);	*/
	}
}

void lpf(float *aX, float *aY, float *aZ)
{
	*aX = (1 - alpha) * filtered_data[0] + alpha * (*aX);
	*aY = (1 - alpha) * filtered_data[1] + alpha * (*aY);
	*aZ = (1 - alpha) * filtered_data[2] + alpha * (*aZ);
	filtered_data[0] = *aX;
	filtered_data[1] = *aY;
	filtered_data[2] = *aZ;
}
