// ---------------------------------------------------------------- //
// CKim - My own kinematics library based on Eigen matrix library
// and Numerical recipes
// Last updated : Oct. 16, 2014 
// ---------------------------------------------------------------- //

#include "ChunKinematics.h"	
//#include "dataManager.h"
#include <iostream>
#include <Eigen/Eigenvalues>


namespace ChunKinematics
{
	double Dot(const Vec4& a, const Vec4& b)
	{
		double sum = 0;
        for(int i=0; i<3; i++)		{	sum += (a(i)*b(i));	}
		return sum;
	}

	Vec4 Cross(const Vec4& a, const Vec4& b)
	{
		Vec4 x;		// Initialize to zero
		x(0) = a(1)*b(2) - a(2)*b(1);		x(1) = a(2)*b(0)-a(0)*b(2);		x(2) = a(0)*b(1)-a(1)*b(0);
        return x;
	}

	Mat4 Skew(const Vec4& w)
	{
		Mat4 x = Mat4::Identity();		// CKim - Initialize to identity
		x(0,1) = -w(2);		x(1,0) = w(2);
		x(0,2) = w(0);		x(2,0) = -w(0);
		x(1,2) = -w(1);		x(2,1) = w(1);
		return x;
	}

	Mat4 Rodrigues(const Vec4& w, const double& th)
	{
		Mat4 R = Mat4::Identity();	// CKim - initializes to identity

		if(w.norm() == 0)	{	return R;	}
		else
		{
			double c = cos(th);		double cc = 1-cos(th);		double s = sin(th);
            R(0,0) = 1 - (w(1)*w(1) + w(2*w(2)))*cc;		R(0,1) = w(0)*w(1)*cc - w(2)*s;				R(0,2) = w(0)*w(2)*cc + w(1)*s;
            R(1,0) = w(0)*w(1)*cc + w(2)*s;					R(1,1) = 1 - (w(0)*w(0)+w(2)*w(2))*cc;		R(1,2) = w(1)*w(2)*cc - w(0)*s;
            R(2,0) = w(0)*w(2)*cc - w(1)*s;					R(2,1) = w(1)*w(2)*cc + w(0)*s;				R(2,2) = 1 - (w(0)*w(0)+w(1)*w(1))*cc;
		}
		return R;
	}

	Vec4 invRodrigues(const Mat4& G)
	{
		Vec4 w = Vec4::Zero();		// CKim - Initializes to zero
		double tr = G.trace();	
		
		if(tr==3)	{	}
		else
		{
			double th = acos( (tr-1)/2 );
			w(0) = (G(2,1) - G(1,2))/(2*sin(th));
			w(1) = (G(0,2) - G(2,0))/(2*sin(th));
			w(2) = (G(1,0) - G(0,1))/(2*sin(th));
			w(3) = th;
		}
		return w;
	}


	Mat4 TwistExp(const Vec6& xi, const double& th)
	{
		Mat4 G = Mat4::Identity();		Mat4 I = Mat4::Identity();
		Vec4 v = Vec4::Zero();			Vec4 w = Vec4::Zero();
		for(int i=0; i<3; i++)	{	v(i) = xi(i);		w(i) = xi(i+3);		}

		if(w.norm() == 0)	{	for(int i=0; i<3; i++)	{	G(i,3) = th*v(i);	}	}
		else
		{
			// CKim - R = exp(w,th)
			G = Rodrigues(w,th);

			// CKim - t = (eye(3)-R)*cross(w,v)+ dot(w,v)*th*w;
			Vec4 t = (I-G)*Cross(w,v);
			for(int i=0; i<3; i++)	{	G(i,3) = t(i) + (Dot(w,v)*th)*w(i);	}
		}
		return G;
	}
	
	Mat4 InvTf(const Mat4& G)
	{
		Mat4 X = Mat4::Identity();	
		for(int i=0; i<3; i++) {
			for(int j=0; j<3; j++)	{	X(i,j) = G(j,i);	X(i,3) -= (G(j,i)*G(j,3));	}	}
		return X;
	}

	Mat6 Adjoint(const Mat4& G)
	{
		Mat6 X;

		// G = [R,t;0,1];	X(1:3,1:3) = X(4:6, 4:6) = G;
		for(int i=0; i<3; i++)	{
			for(int j=0; j<3; j++)	{	X(i,j) = X(i+3,j+3) = G(i,j);	}	}

		// G = [R,t;0,1];	X(1:3,4:6) = hat(t)*R
		for(int i=0; i<3; i++)
		{	
			X(0,3+i) = G(1,3)*G(2,i) - G(2,3)*G(1,i);		
			X(1,3+i) = G(2,3)*G(0,i) - G(0,3)*G(2,i);		
			X(2,3+i) = G(0,3)*G(1,i) - G(1,3)*G(0,i);		
		}

		return X;
	}

    void MatToXYZEuler(const Eigen::Matrix3d& M, Eigen::Vector3d& rxyz)
    {
        rxyz[1] = asin(M(0,2));
        if( M(0,2) == 1 )
        {
            rxyz[2] = 0;
            rxyz[0] = atan2(M(1,0),M(1,1));
        }
        else if( M(0,2) == -1 )
        {
            rxyz[2] = 0;
            rxyz[0] = -atan2(M(1,0),M(1,1));
        }
        else
        {
            rxyz[0] = atan2(-M(1,2),M(2,2));
            rxyz[2] = atan2(-M(0,1),M(0,0));
        }
    }

    void MatToZYZEuler(const Eigen::Matrix3d& M, Eigen::Vector3d& thphigamm)
    {
        thphigamm[1] = acos(M(2,2));
        if( fabs(M(2,2)) == 1 )
        {
            thphigamm[0] = 0;
            thphigamm[2] = atan2(M(1,0),M(0,0));
        }
        else
        {
            thphigamm[0] = atan2(M(1,2),M(0,2));
            thphigamm[2] = atan2(M(2,1),-M(2,0));
        }
    }

    void XYZEulerToMat(const double rx, const double ry, const double rz, Eigen::Matrix3d& M)
    {
        Eigen::Matrix3d Rx;     RotX(Rx,rx);
        Eigen::Matrix3d Ry;     RotY(Ry,ry);
        Eigen::Matrix3d Rz;     RotZ(Rz,rz);

        M = Rx*Ry*Rz;
    }

    void ZYZEulerToMat(const double theta, const double phi, const double gamma, Eigen::Matrix3d& M)
    {
        Eigen::Matrix3d Rth;      RotZ(Rth,theta);
        Eigen::Matrix3d Rphi;     RotY(Rphi,phi);
        Eigen::Matrix3d Rgam;     RotZ(Rgam,gamma);

        M = Rth*Rphi*Rgam;
    }

    void ZYZEulerToMat(const double theta, const double phi, const double gamma, Eigen::Matrix4d& M)
    {
        Eigen::Matrix3d Rth;      RotZ(Rth,theta);
        Eigen::Matrix3d Rphi;     RotY(Rphi,phi);
        Eigen::Matrix3d Rgam;     RotZ(Rgam,gamma);

        M.block<3,3>(0,0) = Rth*Rphi*Rgam;
    }

    void RotX(Eigen::Matrix3d& M, double jA)
    {
        M.setIdentity();
        M(1,1) = cos(jA);       M(1,2) = -sin(jA);
        M(2,1) = sin(jA);       M(2,2) = cos(jA);
    }

    void RotY(Eigen::Matrix3d& M, double jA)
    {
        M.setIdentity();
        M(0,0) = cos(jA);       M(0,2) = sin(jA);
        M(2,0) = -sin(jA);      M(2,2) = cos(jA);
    }

    void RotZ(Eigen::Matrix3d& M, double jA)
    {
        M.setIdentity();
        M(0,0) = cos(jA);       M(0,1) = -sin(jA);
        M(1,0) = sin(jA);       M(1,1) = cos(jA);
    }

    double constrainAngle(double x){
        x = fmod(x,2*acos(-1.0));
        if (x < 0)
            x += (2*acos(-1.0));
        return x;
    }


    bool FindIntersectionOnUnitCircle(const double intv1[2], const double intv2[2], double x[2])
    {
        // Measure the angular distance in [0,pi]
        double d1 = atan2(sin(intv1[1]-intv1[0]),cos(intv1[1]-intv1[0]));
        double d2 = atan2(sin(intv2[1]-intv2[0]),cos(intv2[1]-intv2[0]));
        double dist = atan2(sin(intv2[0]-intv1[0]),cos(intv2[0]-intv1[0]));
        double dd;

        if(dist > d1)   {   return false;   }
        if(dist < 0)
        {
            if(dist < -d2)      {   return false;   }
            if(dist < d1-d2 )
            {
                dd = atan2(sin(intv2[1]-intv1[0]),cos(intv2[1]-intv1[0]));
                x[0] = intv1[0];    x[1] = x[0] + dd;
                return true;
            }
            else    {   x[0] = intv1[0];         x[1] = intv1[1];       return true;        }
        }
        if((dist > 0) && (dist< d1))
        {
            if(dist < d1-d2 )   {   x[0] = intv2[0];    x[1] = intv2[1];    return true;    }
            else
            {
                dd = atan2(sin(intv1[1]-intv2[0]),cos(intv1[1]-intv2[0]));
                x[0] = intv2[0];    x[1] = x[0] + dd;
                return true;
            }
        }
        return false;
    }

//
//	double PadenKahanFirst(const Vec4& w, const Vec4& x, const Vec4& y)
//	{
//		double th;
//
//		if( fabs(Dot(w,x)-Dot(w,y)) > 0.00001 )		{	th = NaN;		}
//		else
//		{
//			Vec4 xp = x - Dot(x,w)*w;		Vec4 yp = y - Dot(y,w)*w;			
//			th = atan2( Dot(w,Cross(xp,yp)), Dot(xp,yp) );
//		}
//		return th;
//	}
//
//	void PadenKahanSecond(const Vec4& w1, const Vec4& w2, const Vec4& x, const Vec4& y, double* th)
//	{
//		Vec4 tmp = Cross(w1,w2);
//		double alp = ( Dot(w1,w2)*Dot(w2,x) - Dot(w1,y) ) / ( SQR(Dot(w1,w2)) - 1 );
//		double bet = ( Dot(w1,w2)*Dot(w1,y) - Dot(w2,x) ) / ( SQR(Dot(w1,w2)) - 1 );
//
//		double gam2 = SQR(x.Norm()) - SQR(alp) - SQR(bet) - 2*alp*bet*Dot(w1,w2);
//		
//		if(gam2 < 0)	{	for(int i=0; i<4; i++)	{	th[i] = NaN;	}		}
//		else
//		{
//			double gam = sqrt(gam2) / tmp.Norm() ;
//	
//			Vec4 z1 = alp*w1 + bet*w2 + gam*tmp;
//			Vec4 z2 = alp*w1 + bet*w2 - gam*tmp;
//
//			th[0] = -PadenKahanFirst(w1,y,z1);		th[1] = PadenKahanFirst(w2,x,z1);
//			th[2] = -PadenKahanFirst(w1,y,z2);		th[3] = PadenKahanFirst(w2,x,z2);
//		}
//	}
//


//    // abc  -   returns se3
//    Vec6 logSE3(const Mat4& G){
//        SE3 T = dataManager::convertEigenToSE3(G);
//        se3 temp = Log(T);
//        Vec6 xi;

//        for (int i = 0; i<6; i++){
//            xi(i) = temp[i];
//        }

//        return xi;

//    }

//    Mat4    forwardKinematics(const std::vector<se3>& twist, const Eigen::Matrix4d& M, const std::vector<double>& joint ){
//        Mat4 T = Mat4::Identity();
//        Mat4 temp;
//        Vec6 xi;

//        std::vector<se3>::const_iterator iter = twist.begin();
//        std::vector<double>::const_iterator iter2 = joint.begin();

//        for(int i = 0; i < twist.size(); i++){
////            xi = dataManager::convertse3ToEigen(*iter);
//            temp = dataManager::convertSE3ToEigen(Exp(*iter, *iter2));
//            T = T*temp;
//            iter++;
//            iter2++;
//        }
//        T = T*M;

//        return T;
//    }

//    bool    solveInvKinMinNorm(std::vector<se3>& twist, const Eigen::Matrix4d& M, const Eigen::Matrix4d& X, const std::vector<double>& joint, Eigen::MatrixXd& sol ){
//        int count = 0;
//        const int MAX_ITER = 10000;
//        const int SPATIAL_DOF = 6;
//        double error = 1e-7;
//        double matNorm;
//        double lambda = 0.001;
//        Eigen::MatrixXd eye = Eigen::MatrixXd::Identity(SPATIAL_DOF,SPATIAL_DOF);
//        Eigen::MatrixXd JJ(SPATIAL_DOF,SPATIAL_DOF);
//        Eigen::MatrixXd jacobian = getJacobian(twist, M, joint);
//        Eigen::EigenSolver<Eigen::MatrixXd> es;
//        Eigen::Matrix4d T = forwardKinematics(twist, M, joint);
//        Eigen::MatrixXd deltaS(SPATIAL_DOF,1);
//        int N = joint.size();
//        Eigen::MatrixXd _joint(N, 1);
//        Eigen::MatrixXd dq(N, 1);
////        eye = Eigen::MatrixXd::Identity();
////        Eigen::MatrixXd _joint(N, 1);
//        vector<double>::const_iterator iter = joint.begin();
//        for(int i = 0; i<N; i++){
//            _joint(i) = *iter;
//            iter++;
//        }

//        do {
//            JJ = jacobian*jacobian.transpose() + lambda*eye;
////            es.compute(JJ, false);
////            cout << "eigenvalues  " << es.eigenvalues() << endl;
//            deltaS = dataManager::convertse3ToEigen(Log(dataManager::convertEigenToSE3(T)/dataManager::convertEigenToSE3(X)));
//            dq = jacobian.transpose()*JJ.llt().solve(deltaS);
//            _joint = _joint - dq;
//            sol = _joint;

//            vector<double> joint_vector;
//            for (int i=0; i<N; i++)
//                joint_vector.push_back(_joint(i));
//            matNorm = (forwardKinematics(twist, M, joint_vector)*X.inverse()).norm();    // Frobenius norm

//            jacobian = getJacobian(twist, M, joint_vector);
//            T = forwardKinematics(twist, M, joint_vector);
//            count++;
//        } while( dq.norm() > error && count < MAX_ITER);

////        cout << "count = " << count << endl;
////        cout << "matNorm = " << matNorm << endl;
////        cout << sol << endl;
//        if ( count < MAX_ITER )
//            return true;
//        else
//            return false;
//    }

//    Eigen::Matrix3d     Skew(const Eigen::Vector3d& w){
//        Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
//        A(1,2) = -w(0);     A(2,1) = w(0);
//        A(0,2) = w(1);      A(2,0) = -w(1);
//        A(0,1) = -w(2);     A(1,0) = w(2);

//        return A;
//    }

//    Eigen::MatrixXd     getJacobian(const std::vector<se3>& twist, const Eigen::Matrix4d& M, const std::vector<double>& joint){
//        const int spatialDoF = 6;
//        int colNum = twist.size();
//        Eigen::MatrixXd jacobian(spatialDoF, colNum);
//        jacobian.setZero();

//        se3         J_i;
//        vector<se3> J_i_list;
//        vector<se3>::const_iterator iter = twist.begin();
//        vector<double>::const_iterator iter2 = joint.begin();
//        SE3         T;
//        T.SetEye();

//        // calculate jacobian
//        J_i_list.push_back(*iter);
//        for (int i = 1; i<colNum; i++){
//            T = T*Exp(*iter, *iter2);
//            iter++;
//            iter2++;
//            J_i_list.push_back(Ad(T, *iter));
//        }

//        // convert jacobian to Eigen
//        iter = J_i_list.begin();
//        for(int i=0; i<colNum; i++){
//            jacobian.block<6,1>(0,i) = dataManager::convertse3ToEigen(*iter);
//            iter++;
//        }

//        return jacobian;
//    }


//    bool    solveInvKinSVD(std::vector<se3>& twist, const Eigen::Matrix4d& M, const Eigen::Matrix4d& X, const std::vector<double>& joint , Eigen::MatrixXd& sol){
//        int count = 0;
//        const int MAX_ITER = 10000;
//        const int SPATIAL_DOF = 6;
//        double error = 1e-7;
//        double matNorm;
//        double lambda = 0.00001;

//        Eigen::MatrixXd jacobian = getJacobian(twist, M, joint);
//        Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
//        Eigen::MatrixXd sigma = svd.singularValues();
//        Eigen::MatrixXd E(SPATIAL_DOF,SPATIAL_DOF);     E.setZero();
//        Eigen::Matrix4d T = forwardKinematics(twist, M, joint);
//        Eigen::MatrixXd deltaS(SPATIAL_DOF,1);
//        int N = joint.size();
//        Eigen::MatrixXd _joint(N, 1);
//        Eigen::MatrixXd dq(N, 1);

//        vector<double>::const_iterator iter = joint.begin();
//        for(int i = 0; i<N; i++){
//            _joint(i) = *iter;
//            iter++;
//        }

//        do {
//            for (int i=0; i<SPATIAL_DOF; i++){
//                E(i,i) = sigma(i)/(sigma(i)*sigma(i) + lambda);
//            }
//            deltaS = dataManager::convertse3ToEigen(Log(dataManager::convertEigenToSE3(T)/dataManager::convertEigenToSE3(X)));
//            dq = svd.matrixV()*E*svd.matrixU().transpose()*deltaS;
//            _joint = _joint - dq;
//            sol = _joint;

//            vector<double> joint_vector;
//            for (int i=0; i<N; i++)
//                joint_vector.push_back(_joint(i));
//            matNorm = (forwardKinematics(twist, M, joint_vector)*X.inverse()).norm();    // Frobenius norm
//            jacobian = getJacobian(twist, M, joint_vector);
//            Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
//            sigma = svd.singularValues();
//            T = forwardKinematics(twist, M, joint_vector);
//            count++;
//        } while( dq.norm() > error && count < MAX_ITER);

//        if ( count < MAX_ITER )
//            return true;
//        else
//            return false;
//    }
}






























