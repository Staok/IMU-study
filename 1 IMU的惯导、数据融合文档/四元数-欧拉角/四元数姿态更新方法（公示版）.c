/*
1主函数在最后面
2作者：黑市，黑视，智涅
3四元数更新用的是一阶算法，还有二阶三阶甚至全阶，阶数越高精度越好，不过没多大必要。
	一阶二阶这些简化算法就是用简单的值取代了一些三角函数而已
4详细书籍可以看《捷联式惯性导航原理》，袁信著。我以前看了这本书PDF版好久了，估计一半还没能吃透。
5关于利用加速度计来修正姿态，大家貌似都是做飞机的，飞机上的加速度计的情况跟我做的东西差别太大，
应该不能直接引用我那一套，大家还是引用网上例如权重法来进行修正吧！用罗盘修正姿态就更不用说了~
*/
#define EulerAngle_Type float	//定义类型
#define Quaternion_Type float
#define Acc_Type int
#define Gyro_Type int
#define Euler_Martix_Type float

struct EulerAngle		//欧拉角结构体
{
	EulerAngle_Type Roll, Pitch, Yaw;
}
struct Quaternion		//四元数结构体
{
	Quaternion_Type q0, q1, q2, q3;
}
struct Acc		//加速度值结构体
{
	Acc_Type x, y, z;
}
struct Gyro		//陀螺仪值结构体
{
	Gyro_Type x, y, z;
}
struct Euler_Martix		//欧拉（姿态）矩阵结构体
{
	Euler_Martix_Type T11,T12,T13,	T21,T22,T23,	T31,T32,T33;
}

Quaternion Normalize(Quaternion e)		//四元数归一化
{
	Quaternion_Type s = (Quaternion_Type)Math.Sqrt(e.q0 * e.q0 + e.q1 * e.q1 + e.q2 * e.q2 + e.q3 * e.q3);
	e.q0 /= s;
	e.q1 /= s;
	e.q2 /= s;
	e.q3 /= s;
	return e;
}

Quaternion Multiply_L1(Acc lacc)		//一阶算法
{
	Quaternion Q_result;
	Q_result.q0 = BQ.q0 - BQ.q1 * lacc.x / 2 - BQ.q2 * lacc.y / 2 - BQ.q3 * lacc.z / 2;
	Q_result.q1 = BQ.q1 + BQ.q0 * lacc.x / 2 + BQ.q2 * lacc.z / 2 - BQ.q3 * lacc.y / 2;
	Q_result.q2 = BQ.q2 + BQ.q0 * lacc.y / 2 - BQ.q1 * lacc.z / 2 + BQ.q3 * lacc.x / 2;
	Q_result.q3 = BQ.q3 + BQ.q0 * lacc.z / 2 + BQ.q1 * lacc.y / 2 - BQ.q2 * lacc.x / 2;
	Q_result = Quaternion_Normalize(Q_result);
	return Q_result;
}

Euler_Martix Q_to_EM(Quaternion e)		//把四元数变换成欧拉角（姿态）矩阵T
{
	Euler_Martix result;
	Euler_Martix_Type q00,q01,q02,q03,q11,q12,q13,q22,q23,q33;
	q00=e.q0*e.q0;
	q01=e.q0*e.q1;
	q02=e.q0*e.q2;
	q03=e.q0*e.q3;
	q11=e.q1*e.q1;
	q12=e.q1*e.q2;
	q13=e.q1*e.q3;
	q22=e.q2*e.q2;
	q23=e.q2*e.q3;
	q33=e.q3*e.q3;
	result.T11=q00+q11-q22-q33;
	result.T12=2*(q12+q03);
	result.T13=2*(q13-q02);
	result.T21=2*(q12-q03);
	result.T22=q22-q33+q00-q11;
	result.T23=2*(q23+q01);
	result.T31=2*(q13+q02);
	result.T32=2*(q23-q01);
	result.T33=q33-q22-q11+q00;
	return result;
}

Quaternion Ea_to_Qu(EulerAngle ea)		//把欧拉角变换成四元数      后来不用这个方法了，用矩阵那个了
{
	Quaternion result;

	Quaternion_Type CosY = Math.Cos(ea.Yaw * .5);
	Quaternion_Type SinY = Math.Sin(ea.Yaw * .5);	
	Quaternion_Type CosP = Math.Cos(ea.Pitch * .5f);
	Quaternion_Type SinP = Math.Sin(ea.Pitch * .5);
	Quaternion_Type CosR = Math.Cos(ea.Roll * .5f);
	Quaternion_Type SinR = Math.Sin(ea.Roll * .5f);
	
	result.q0 = CosY * CosP * CosR + SinY * SinP * SinR;
	result.q1 = CosY * CosP * SinR - SinY * SinP * CosR;
	result.q2 = CosY * SinP * CosR + SinY * CosP * SinR;
	result.q3 = SinY * CosP * CosR - CosY * SinP * SinR;
	return result;
}

Acc	coordinate_body_to_inertia(Euler_Martix EM,Acc lacc)		//将体坐标加速度变换到惯性坐标
{
//做飞机不需要，省略
}

EulerAngle EM_to_EU(Euler_Martix lem)        //从姿态矩阵中提取姿态角
{
	EulerAngle result;
	result.Yaw = Math.Atan2(lem.T12, lem.T11);
	result.Pitch = -Math.Asin(lem.T13);
	result.Roll = Math.Atan2(lem.T23, lem.T33);
	return result;
}

void main()
{
	Quaternion BQ;//定义姿态四元素
	Euler_Martix BEM;//定义欧拉矩阵
	EulerAngle BEA;//定义欧拉角
	
	BQ.q0=1;//初始化四元数
	BQ.q1=BQ.q2=BQ.q3=0;//初始化四元数

	char gx,gy,gz;//定义陀螺仪三个轴，用来装值
	
	while（1）//要不断更新，所以弄个循环，你要知道你的更新速率，才能转化下面的一些参数。
			//根据转动的不可交互性，更新速率越快越好，看书！
	{
		gx=25;
		gy=14;
		gz=4;
		//以上为读取陀螺仪的三个值，我随便赋值作为例子哈
		
		EulerAngle lea;//定义欧拉小转角，每次更新的小转角，下面就用到
		
		lea.Yaw = ((float)gx / 62200 * Math.PI);
		lea.Pitch = ((float)gy / 62200 * Math.PI);
		lea.Roll = ((float)gz/ 62200 * Math.PI);
		//把陀螺仪的三个值转化为角度值（根据自己的采样率、精度等参数转化），不懂请请自行查资料搞懂
		
		BQ = Multiply_L1(BQ, lea);//更新姿态四元素，由旧四元数和小转角更新得到新四元数
		
		BEM = Q_to_EM(BQ);//将更新完的姿态四元素转成欧拉矩阵
		BEA = EM_to_EU(BEM);//从欧拉矩阵中提取欧拉角
	}
}

