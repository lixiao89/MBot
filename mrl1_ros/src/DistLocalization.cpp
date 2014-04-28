#include "DistLocalization.hpp"

std_msgs::Float64 DistLocalization::velocityPID(float velDesire, float currVel, float kp, float ki, float kd)
{
    float p_error; // difference b/w desired vel and current vel
    float i_error; // sum of errors over time
    float d_error; // difference b/w previous and curr proportional error

    
    float p_out; // proportional component of output
    float i_out;
    float d_out;

    float output; // sum of PID outputs

    // Calculate three errors
    p_error = velDesire - currVel;
    i_error = integral_error;
    d_error = p_error - previous_error;

    // calcualte the three output components
    p_out = kp*p_error;
    i_out = ki*i_error;
    d_out = kd*d_error;

    output = p_out + i_out + d_out;
   
    if(output > 15 || output < -15)
    {
        output = 3;
    }

    //cout << output << endl;

    previous_error = p_error;
    integral_error += p_error;

    std_msgs::Float64 jointEffort;

    jointEffort.data = output;

    return jointEffort;
}

void DistLocalization::moveToLongestScan()
{
    
    int minRangeIndex = 0;
    float minRangeTemp = maxRange; 

    for(int i=0; i < rangeReadings.size();i++)
    {
        if(rangeReadings.at(i) < minRangeTemp)
        {
            minRangeTemp = rangeReadings.at(i);
            minRangeIndex = i;
        }
    }           

    if(angleReadings.at(minRangeIndex) > 0)
    {
        leftEffort.data = 2;
        rightEffort.data = -2;
    }
    else if(angleReadings.at(minRangeIndex) < 0)
    {
        leftEffort.data = -2;
        rightEffort.data = 2;

    }
    else
    {
        leftEffort.data = 3;
        rightEffort.data = 3;
    }

}

void DistLocalization::moveSinasoidal()
{   
    ros::Time time = ros::Time::now();

    float t;
    t = time.toSec();

    leftEffort.data = sin(t)+1;
    rightEffort.data = sin(t)+0.5;
}

//////////////////// Distributed EKF ///////////////////////
//

void DistLocalization::distEKF()
{

        if(w1<0.01 || w2<0.01)
        {
            
            w2 = encoderLeftVel;
            w1 = encoderRightVel;
            return;
        }


    double l = 0.30;
    double r = 0.08;
  
cout<<robot<<":"<<w1<<","<<w2<<endl;
cout<<robot<<":"<<poseCurr(1-1)<<","<<poseCurr(2-1)<<","<<poseCurr(3-1)<<endl;
cout<<robot<<"selfGT:"<<selfGT(0)<<","<<selfGT(1)<<","<<selfGT(2)<<endl;
cout<<robot<<"neighborGT:"<<neighborGT(0)<<","<<neighborGT(1)<<","<<neighborGT(2)<<endl;

        posEsti.poseEst.x = poseCurr(1-1);
        posEsti.poseEst.y = poseCurr(2-1);
        posEsti.poseEst.theta = poseCurr(3-1);
        

        int k=0;
        for(int i =0; i < 3; i++)
            for(int j=0; j<3; j++)
        {
             posEsti.cov[k] = stateCovCurr(i,j);
             k=k+1;
        }

      poseEstimate_.publish(posEsti);




if(w1 == 0 && w2 == 0 )
{
    Vim = (w1+w2)*r/2;
    Wim = (w1-w2)*r/l;

//predict
this->distEKFPred(poseCurr,stateCovCurr, Vim, Wim);


}
else
{

              


    Vim = (w1+w2)*r/2;
    Wim = (w1-w2)*r/l;

//predict
this->distEKFPred(poseCurr,stateCovCurr, Vim, Wim);



          Eigen::Vector3d relativePose;
 

        float minRangeAd = minRange + 0.061;
        float minAngleAd = minAngle +0.01;
        float x_relative = minRangeAd*cos(minAngleAd);
        float y_relative = minRangeAd*sin(minAngleAd);


        float pi = 3.14159265359;
        float dtheta = pi-(pi-abs(minAngle)+(pi-abs(nminAngle)));
        float theta_relative = copysign(dtheta,minAngle);


Eigen::Vector3d z(x_relative,y_relative,theta_relative);
//cout<<robot<<"z:"<<z(0)<<","<<z(1)<<","<<z(2)<<endl;



//update
this->distEKFUpdate(poseCurr, stateCovCurr,z);

}

      w2 = encoderLeftVel;
      w1 = encoderRightVel;



       }

  // prediction 
       void DistLocalization::distEKFPred(Eigen::Vector3d& poseCurr,Eigen::Matrix3d& stateCovCurr, double Vim, double Wim)
{
    ros::Duration dt = currTime - previousTime;

    double ddt = dt.toSec();
    ddt = abs(ddt);
    
    if((ddt > 0 && ddt < 0.005) || ddt > 1)
    {
        ddt = 1/freq;
    }



   Eigen::Matrix3d Phi;
   Eigen::MatrixXd G(3,2);

      Phi << 1,0,-Vim*ddt*sin(poseCurr(3-1)),
             0,1,Vim*ddt*cos(poseCurr(3-1)),
             0,0,1;

     G << ddt*cos(poseCurr(3-1)), 0,
          ddt*sin(poseCurr(3-1)), 0,
          0,ddt;

    Eigen::Matrix3d Qdi;
    Qdi = G*processNoiseCov*G.transpose();

    // propagate state covariance
    stateCovCurr = Phi*stateCovCurr*Phi.transpose()+Qdi; 

      // propagate state
      poseCurr(1-1) = poseCurr(1-1)+Vim*ddt*cos(poseCurr(3-1));
      poseCurr(2-1) = poseCurr(2-1)+Vim*ddt*sin(poseCurr(3-1));
      poseCurr(3-1) = poseCurr(3-1)+Wim*ddt;

      previousTime = currTime;
   }


// update
 void DistLocalization::distEKFUpdate(Eigen::Vector3d& poseCurr,Eigen::Matrix3d& stateCovCurr,Eigen::Vector3d z)
{
        // use real neighbor position (GPS etc)
       // subscribedX = neighborGT(1-1);
       // subscribedY = neighborGT(2-1);
       // subscribedTheta = neighborGT(3-1)-3.14159265359/2;

        Eigen::Matrix2d C,J;
        C << cos(poseCurr(3-1)),-sin(poseCurr(3-1)),
             sin(poseCurr(3-1)), cos(poseCurr(3-1));

         J << 0,-1,
              1,0;

        Eigen::Matrix3d Gamma;
        Gamma << cos(poseCurr(3-1)),-sin(poseCurr(3-1)),0,
                 sin(poseCurr(3-1)), cos(poseCurr(3-1)),0,
                 0,0,1;

        Eigen::Vector2d v_self(poseCurr(1-1),poseCurr(2-1)), v_diff, v_temp;

        Eigen::Vector2d v_sub(subscribedX,subscribedY);
        
        //using real neighbor position
        //Eigen::Vector2d v_sub(neighborGT(1-1),neighborGT(2-1));

        v_diff = v_sub - v_self;
        v_temp = J*v_diff;

        Eigen::Matrix3d tH;

        tH << 1,0,v_temp(1-1),
              0,1,v_temp(2-1),
              0,0,1;

        Eigen::Matrix3d tS, tR, I ;
        I = Eigen::MatrixXd::Identity(3,3);
        tR = Gamma*cov_m*Gamma.transpose();
        tS = tH*stateCovCurr*tH.transpose()+subscribedCov+tR;


        Eigen::Vector3d v_neighbor(subscribedX,subscribedY,subscribedTheta);

// using real neighbor position
        //Eigen::Vector3d v_neighbor(neighborGT(1-1),neighborGT(2-1),neighborGT(3-1));


        //update pose
        poseCurr = (I-stateCovCurr*tH.transpose()*tS.inverse())*poseCurr + stateCovCurr*tH.transpose()*tS.inverse()*(v_neighbor-Gamma*z);

        //update state covariance
        stateCovCurr = stateCovCurr-stateCovCurr*tH.transpose()*tS.inverse()*tH*stateCovCurr;


}



///////////////// Exponential Localization /////////////////

 // Main method to call
     void DistLocalization::expLocalization()
    {
        if(w1<0.01 || w2<0.01)
        {
            
            w2 = encoderLeftVel;
            w1 = encoderRightVel;
            return;
        }

       ros::Duration dt = currTime - previousTime;

        double ddt = dt.toSec();
        ddt = abs(ddt);

        if((ddt > 0 && ddt < 0.005) || ddt > 1)
        {
            ddt = 1/freq;
        } 


            cout<<robot<<":"<<w1<<","<<w2<<endl;
            
            // current pose of the robot
            currPose = ExpMath::SE2ToXYTheta(a_i);
    
//cout<<robot<<":"<<"pose:"<<currPose(0)<<","<<currPose(1)<<endl;

        posEsti.poseEst.x = currPose(1-1);
        posEsti.poseEst.y = currPose(2-1);
        posEsti.poseEst.theta = currPose(3-1);
        
         estError.x = currPose(1-1) - selfGT(1-1);
         estError.y = currPose(2-1) - selfGT(2-1);
         estError.theta = currPose(3-1) - selfGT(3-1);


        int k=0;
        for(int i =0; i < 3; i++)
            for(int j=0; j<3; j++)
        {
             posEsti.cov[k] = cov_i(i,j);
             k=k+1;
        }
      

        estError_.publish(estError);
        poseEstimate_.publish(posEsti);
        

            // variables used for the prediction step
            Eigen::Matrix3d mu_pred_pre;
            Eigen::Matrix3d cov_pred_pre;

            Eigen::Matrix3d mu_pred_post;
            Eigen::Matrix3d cov_pred_post;

            // prediction step 1
            //
            this->SDEPrediction(w1, w2, mu_pred_pre, cov_pred_pre);
 
//double t1;
//t1 = atan2(mu_pred_pre(2-1,1-1),mu_pred_pre(2-1,2-1));
//cout<<robot<<"t1="<<t1<<endl;


           // prediction setp 2
            
            ExpMath::convolutionSE2(a_i, mu_pred_pre,cov_i,cov_pred_pre,mu_pred_post,cov_pred_post);
//double t2;
//t2 = atan2(mu_pred_post(2-1,1-1),mu_pred_post(2-1,2-1));
//cout<<robot<<"t2="<<t2<<endl;
           
// ------------- finding the measurement matrix mu_m -----------

   {
         Eigen::Vector3d relativePose;
 

        float minRangeAd = minRange + 0.061;
        float minAngleAd = minAngle - 0.05;
        float x_relative = minRangeAd*cos(minAngleAd);
        float y_relative = minRangeAd*sin(minAngleAd);

        float pi = 3.14159265359;
        float dtheta = pi-(pi-abs(minAngle)+(pi-abs(nminAngle)));
        float theta_relative = copysign(dtheta,minAngle);

        relativePose<< x_relative,y_relative,theta_relative;

       cout<<robot <<":"<<relativePose(0)<<","<<relativePose(1)<<","<<relativePose(2)<<endl;



        Eigen::Matrix3d mu_m;
        mu_m = ExpMath::XYThetaToSE2(relativePose);

//---------------------------------------------
       

// calculate the SE2 representation of position of neighborPos
        Eigen::Vector3d neighborPos;

        {

        // use real neighbor position
       // subscribedX = neighborGT(1-1);
       // subscribedY = neighborGT(2-1);
       // subscribedTheta = neighborGT(3-1)-3.14159265359/2;

        neighborPos<<subscribedX,subscribedY,subscribedTheta;
        }


//cout<<robot<<":"<<neighborPos(0)<<","<<neighborPos(1)<<","<<neighborPos(2)<<endl;

        Eigen::Matrix3d mu_j;
        mu_j = ExpMath::XYThetaToSE2(neighborPos);


        Eigen::Matrix3d cov_j;
        cov_j = subscribedCov;
       

        // update step
        Eigen::Matrix3d I;
        I = Eigen::MatrixXd::Identity(3,3);


         this->fusion_with_sensor_noise(I, mu_pred_post, cov_pred_post, a_j, mu_j, cov_j, mu_m, cov_m, mu_i_bar, Sigma_i_bar);
//double t3;
//t3 = atan2(mu_i_bar(2-1,1-1),mu_i_bar(2-1,2-1));
//cout<<"t3="<<t3<<endl;


        Eigen::Vector3d X_i_bar;
        X_i_bar = ExpMath::SE2ToXYTheta(mu_i_bar);

        a_i = mu_i_bar;
        cov_i = Sigma_i_bar;

 }
/*{
        Eigen::Vector3d X_i_bar;
        X_i_bar = ExpMath::SE2ToXYTheta(mu_pred_post);
       


        a_i = mu_pred_post;
        cov_i = cov_pred_post;

}*/
            w2 = encoderLeftVel;
            w1 = encoderRightVel;
            

            //
        

    }


 void DistLocalization::SDEPrediction(float w1, float w2, Eigen::Matrix3d& mu, Eigen::Matrix3d& cov)
{
    double l = 0.3;
    double r = 0.08;
    double D = 1;

    ros::Duration dt = currTime - previousTime;

    double ddt = dt.toSec();
    ddt = abs(ddt);

    if((ddt > 0 && ddt < 0.005) || ddt > 1)
    {
        ddt = 1/freq;
    }

    
    cout<<"ddt:"<<ddt<<endl;
    /*if(abs(dcurrTime - dpreviousTime) < 0.005 || abs(dcurrTime - dpreviousTime) >1)
    {
        dpreviousTime = dcurrTime - 0.5;
    }*/

// ----------------------  Calculating mu --------------------------------------

    /*double mu11 = 1 - (pow(r,2)*pow(ddt,2)*pow(w1-w2,2)/(2*pow(l,2)));
    double mu12 = r*ddt*(w1-w2)/l;
    double mu13 = r*ddt*(w1+w2)/2;
    double mu23 = pow(r,2)*pow(ddt,2)*(pow(w1,2)-pow(w2,2))/(4*l);

    mu << mu11,-mu12, mu13,
          mu12, mu11, mu23,
          0,0,1;*/

    Eigen::Vector3d h(r*ddt*(w1+w2)/2,0,r*ddt*(w1-w2)/l);

    Eigen::Matrix3d hhat;
    hhat = ExpMath::wedge(h);

    mu = hhat.exp();


// ---------------------- Calculating cov -----------------------------------

    symbol t("t");

    float c1 = pow(w1,2) - pow(w2,2);
    ex c8 = 2*pow(r,2)*pow(t,2)*w1*w2;
    ex c9 = pow(r,2)*pow(t,2)*pow(w2,2);
    ex c10 = pow(r,2)*pow(t,2)*pow(w1,2);
    ex c6 = 2*pow(l,2) - c10 + c8 - c9;
    ex c7 = 4*pow(l,4) + pow(r,4)*pow(t,4)*pow(w1,4) - 4*pow(r,4)*pow(t,4)*pow(w1,3)*w2 + 6*pow(r,4)*pow(t,4)*pow(w1,2)*pow(w2,2) - 4*pow(r,4)*pow(t,4)*w1*pow(w2,3) + pow(r,4)*pow(t,4)*pow(w2,4);
     //ex c7 = 4*pow(l,4) - 4*pow(r,4)*pow(t,4)*w1*pow(w2,3);
 
    ex c2 = 2*pow(l,2) + c10 - c8 + c9;
    ex c3 = (4*pow(l,5)*pow(r,3)*t*D*(w1-w2)*c6)/pow(c7,2);
    ex c4 = (4*pow(l,3)*pow(r,3)*t*D*(w1+w2))/(l*c7);
    ex c5 = l*pow(c7,2);


   // ex c11 = (2*pow(l,4)*D*pow(r,2)*pow(c6,2)/pow(c7,2)) + (l*pow(r,6)*pow(t,4)*D*(w1+w2)*c1*(w1-w2)*pow(c2,2)/(2*l*pow(c7,2)));
     ex c11 = (2*pow(l,4)*D*pow(r,2)*pow(c6,2)/pow(c7,2)) + (l*pow(r,6)*pow(t,4)*D*(w1+w2)*c1*(w1-w2)*pow(c2,2)/(2*l*pow(c7,2)));
 
    ex c12 = -c3 + (2*pow(l,4)*pow(r,5)*pow(t,3)*D*(w1+w2)*c1*c2/c5);
    ex c22 = (8*pow(l,6)*pow(r,2)*pow(t,2)*D*pow(r,2)*(pow(w1,2)*l+pow(w2,2)*l+l*pow(w1,2)+l*pow(w2,2)))/c5;
    ex c13 = pow(r,2)*pow(t,2)*D*pow(r,2)*c1*c2/(l*c7);
    ex c23 = c4;
    double c33 = 2*D*pow(r,2)/pow(l,2);

    previousTime = currTime;

    //----------
    double res11 = ExpMath::SimpsonIntegrate(t,c11,0,ddt);
    double res12 = ExpMath::SimpsonIntegrate(t,c12,0,ddt);
    double res13 = ExpMath::SimpsonIntegrate(t,c13,0,ddt);
    double res22 = ExpMath::SimpsonIntegrate(t,c22,0,ddt);
    double res23 = ExpMath::SimpsonIntegrate(t,c23,0,ddt);
    double res33 = c33*ddt;

    cov << res11, res12, res13,
           res12, res22, res23,
           res13, res23, res33;
    }


	

	    // returns xi and Si
void DistLocalization::generate_Si(Eigen::Matrix3d Sigma_i, Eigen::Vector3d& xi, Eigen::Matrix3d& Si)
{
    xi = GroupMathSE::ExpMath::SE2ToExp(Eigen::MatrixXd::Identity(3,3));
    Eigen::Matrix3d Xi;
    Xi = GroupMathSE::ExpMath::wedge(xi);

    Eigen::Matrix3d gamma_i;
    gamma_i = (Eigen::MatrixXd::Identity(3,3) + 0.5*GroupMathSE::ExpMath::SE2_ad(Xi));

    Si = gamma_i.transpose()*Sigma_i.inverse()*gamma_i;
}   


       
        // returns xm and Sm
void DistLocalization::generate_Sm(Eigen::Matrix3d m_im,Eigen::Matrix3d mu_m,Eigen::Matrix3d a_m,Eigen::Matrix3d a_i,Eigen::Matrix3d mu_i,Eigen::Matrix3d Sigma_m,Eigen::Vector3d& xm,Eigen::Matrix3d& Sm)
{
    Eigen::Matrix3d qm;
    qm = m_im*mu_m.inverse()*a_m.inverse()*a_i*mu_i;
 
    xm = GroupMathSE::ExpMath::SE2ToExp(qm);

    Eigen::Matrix3d Xm;
    Xm = GroupMathSE::ExpMath::wedge(xm);

    Eigen::Matrix3d gamma_m;
    gamma_m = (Eigen::MatrixXd::Identity(3,3)+0.5*GroupMathSE::ExpMath::SE2_ad(Xm));
 
    Eigen::Matrix3d temp;
    temp = GroupMathSE::ExpMath::SE2_Adjoint(m_im);
    temp = temp.inverse().eval();

    Sm = gamma_m.transpose()*temp.transpose()*Sigma_m.inverse()*temp*gamma_m;
}


void DistLocalization::Fusion(Eigen::Matrix3d a_i,Eigen::Matrix3d mu_i,Eigen::Matrix3d cov_i,Eigen::Matrix3d A_m,Eigen::Matrix3d MU_m,Eigen::Matrix3d COV_m,Eigen::Matrix3d Mim, Eigen::Matrix3d& mu_i_bar, Eigen::Matrix3d& Sigma_i_bar)
{
    Eigen::Vector3d xi(0,0,0);
    Eigen::Matrix3d Si;
    Si = cov_i.inverse();


    Eigen::Vector3d xm;
    Eigen::Matrix3d Sm;

   this->generate_Sm(Mim,MU_m,A_m,a_i,mu_i,COV_m,xm,Sm);


    Eigen::Vector3d xbar;
    xbar = (Sm+Si).inverse()*(Si*xi+Sm*xm);

//cout<<Si<<endl<<xi<<endl<<Sm<<endl<<xm<<endl;
//cout<<Sm.inverse()*Si*xi<<endl<<Sm.inverse()*Sm*xm<<endl;

    Eigen::Matrix3d Xbar;
    Xbar = GroupMathSE::ExpMath::wedge(xbar);


    Eigen::Matrix3d gamma_bar;

    gamma_bar = (Eigen::MatrixXd::Identity(3,3)+0.5*GroupMathSE::ExpMath::SE2_ad(Xbar));

    Sigma_i_bar = gamma_bar*(Si+Sm).inverse()*gamma_bar.transpose();
   
    Eigen::Matrix3d temp1, temp2;
    temp1 = (-Xbar).exp();
    temp2 = Eigen::MatrixXd::Identity(3,3)-Xbar+0.5*Xbar*Xbar-0.1666667*Xbar*Xbar*Xbar+0.04166667*Xbar*Xbar*Xbar*Xbar;
  
    mu_i_bar = mu_i*((-Xbar).exp());

    int i;

}


        // returns the final estimate update for mean (mu_i_bar) and covariance (Sigma_i_bar)
         void DistLocalization::fusion_with_sensor_noise(Eigen::Matrix3d a_i,Eigen::Matrix3d mu_i,Eigen::Matrix3d cov_i,Eigen::Matrix3d a_j,Eigen::Matrix3d mu_j,Eigen::Matrix3d cov_j,Eigen::Matrix3d mu_m,Eigen::Matrix3d cov_m,Eigen::Matrix3d& mu_i_bar,Eigen::Matrix3d& Sigma_i_bar)
{
    /* mu_i and cov_i are 3x3 matrices representing the estimated mean and covariance of the robot to be updated

         a_i is a 3x3 matrix denoting the initial position of robot i in world frame

        mu_i is a 3x3 matrix representing the mean of robot i relative to a_i

         a_j, mu_j, cov_j are the 3x3 matrices of robot j (robot to take measurement from)

         mu_m and cov_m are 3x3 the mean and covariance of the measurement  distribution (sensor model and gaussian in this case)*/

    // --------------- Perform the Convolution-like Calculation ------------------

    // Basis elements for SE(2)


    double E_basis[3][3][3]=
    {
        {
            {0,0,1},
            {0,0,0},
            {0,0,0},
        },

        {
            {0,0,0},
            {0,0,1},
            {0,0,0},
        },

        {
            {0,-1,0},
            {1,0,0},
            {0,0,0},
        },

    };


    Eigen::Matrix3d mu2;
    mu2 = a_j*mu_j;

    Eigen::Matrix3d A,B,C,D,E,temp;
    temp = GroupMathSE::ExpMath::SE2_Adjoint(mu2.inverse());
    A =temp*cov_j*temp.transpose();
    B = cov_m;


    C = Eigen::MatrixXd::Zero(3,3);
    D = Eigen::MatrixXd::Zero(3,3);
    E = Eigen::MatrixXd::Zero(3,3);

    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {


            Eigen::Matrix3d Ei,Ej;

            Ei = GroupMathSE::ExpMath::mapArray3D(E_basis,i);
            Ej = GroupMathSE::ExpMath::mapArray3D(E_basis,j);


            Eigen::Matrix3d adi,adj;
            adi = GroupMathSE::ExpMath::SE2_ad(Ei);
            adj = GroupMathSE::ExpMath::SE2_ad(Ej);

            C = C + adi*B*adj.transpose()*A(i,j);
            D = D + adi*adj*A(i,j)*B + (adi*adj*A(i,j)*B).transpose();
            E = E + adi*adj*B(i,j)*A + (adi*adj*B(i,j)*A).transpose();
        }
    }

    C = C/4;
    D = D/12;
    E = E/12;

    Eigen::Matrix3d cov_nin;
    cov_nin = A + B + C + D + E;

    
    this->Fusion(a_i,mu_i,cov_i,a_j,mu_j,cov_nin,mu_m,mu_i_bar,Sigma_i_bar);
    
}

