#include "Buff_Task.h"
struct timeval sin_time_next;
extern mutex mtx;
bool XML_For_Prediction = false;
extern double ****SinDataSheet;
bool UpdateSin = false;
extern bool Predict_Flag;
extern int UpdateTimes;

void Buff_Class::prediction(){
    // xcorr_fft(N,fs);
    xcorr_sub();
    

}
void Buff_Class::xcorr_sub()
{
    mtx.lock();
    int v_size = Buff.v_data.size();
    if(v_size>(0) && Predict_Flag)
    {
        Predict_Flag = false;
        vector<struct timeval> v_time=Buff.v_time;
        vector<double>v_data=Buff.v_data;

        mtx.unlock();   

        //限幅平均滤波
        int FILTER_N =15;
        double FILTER_A=1.5;
        for(int i = FILTER_N; i < v_size - 1; i++) 
        {
        
            if(((v_data[i - 1] - v_data[i - 2]) > FILTER_A) || ((v_data[i - 2] - v_data[i - 1]) > FILTER_A))
            {
            v_data[i - 1] = v_data[i - 2];
            double filter_sum = 0;
            for(int j = i - FILTER_N -1 +1; j < i; j++) {
                v_data[j] = v_data[j + 1];
                if(v_data[j]>3.0){
                // fft_in[j][0]=0;
                    filter_sum+=1.5;
                }
               else
                    filter_sum += v_data[j];
            }
                if(filter_sum / (FILTER_N - 1)>3.0)
                    v_data[i - 1]=3.0;
                else if(filter_sum / (FILTER_N - 1)<0.0)
                    v_data[i - 1]=0.0;
                else
                    v_data[i - 1]=filter_sum / (FILTER_N - 1);
                
            }

        }  

        double v_mean =  std::accumulate(std::begin(v_data), std::end(v_data), 0.0) / double(v_size); //计算速度均值
        SinData Sin_out;
        // double min_f = 0.29,max_f = 0.325;//0.3  0.31831
        // double min_A = 0.770,max_A = 1.055;//0.780  1.045
        double min_C = 2.090 - min_A, max_C = 2.090 - max_A;
        double min_offset = 1e+5;
        if(XML_For_Prediction)//是否使用xml的数据进行预测 不使用xml的话 可以不管
        {
            for (int i = 0; min_f + double(i) * interval_f < max_f; i++)
            {
                SinData Sin_in;
                Sin_in.sin_f = min_f+double(i) * interval_f;
                Sin_in.sin_A = 0;
                Sin_in.sin_C = v_mean;
                for (int j = 0; min_A + double(j) * interval_A < max_A; j++)
                {
                    Sin_in.sin_A = min_A + double(j) * interval_A;
                    if(v_mean < min_C)
                        v_mean = min_C;
                    else if (v_mean > 1.395) //1.395
                    {
                        v_mean=1.395;
                    }
                    for(int k = 0;k < Theta_Size;k++)
                    {
                        double Curr_sub_vla = 0;
                        for (int n = 0; n < v_size && n<=300; n++)//n<=300 是为了减少运算量
                        {
                            double Signal_temp = SinDataSheet[i][j][k][n] + Sin_in.sin_C;
   
                            Curr_sub_vla += abs(v_data[n]-Signal_temp);//累加偏差           
                        }    
                        if(min_offset >= Curr_sub_vla)//偏差最小的曲线
                        {
                            min_offset = Curr_sub_vla;
                            Sin_out.sin_A = Sin_in.sin_A;
                            Sin_out.sin_f = Sin_in.sin_f;
                            Sin_out.sin_C = Sin_in.sin_C;
                            Sin_out.sin_theta = double(k)*2.0*PI/Theta_Size;
                        }
                    }
                }
            }
        }
        else
        {
            for (int i = 0; min_f + double(i) * interval_f < max_f; i++)
            {
                SinData Sin_in;
                Sin_in.sin_f = min_f+double(i)*interval_f;
                Sin_in.sin_A = 0;
                Sin_in.sin_C = v_mean;
                for (int j = 0; min_A + double(j)*interval_A < max_A; j++)
                {
                    Sin_in.sin_A = min_A + double(j)*interval_A;
                    if(v_mean < min_C)
                        v_mean = min_C;
                    else if (v_mean > 1.395) //1.395
                    {
                        v_mean=1.395;
                    }
                    for(int k = 0;k < Theta_Size;k++)
                    {
                        double Curr_sub_vla = 0;
                        for (int n = 0; n < v_size && n<=300; n++)//n<=300 是为了减少运算量
                        {
                            double Signal_temp = Sin_in.sin_A * sin(Sin_in.sin_f * 2 * PI * (double(n)/double(fs)*double(num_mean)) +
                            double(k)*2.0*PI/Theta_Size) + Sin_in.sin_C;
                            Curr_sub_vla += abs(v_data[n]-Signal_temp);     //累加偏差                        
                        }    
                        if(min_offset >= Curr_sub_vla)//偏差最小的曲线
                        {
                            min_offset = Curr_sub_vla;
                            Sin_out.sin_A = Sin_in.sin_A;
                            Sin_out.sin_f = Sin_in.sin_f;
                            Sin_out.sin_C = Sin_in.sin_C;
                            Sin_out.sin_theta = double(k)*2.0*PI/Theta_Size;
                        }
                    }
                }
            }
        }
        //频率太快云台会抖
        if(v_size < 50)
        {
            if(UpdateTimes % 10 == 0)//每10次更新一次曲线
                UpdateSin = true;
        }else
        {
            UpdateSin = true;

        }
        if(UpdateSin || (v_size>5 &&v_size<10))
        {
            mtx.lock();
            Buff.sin_A = Sin_out.sin_A;
            Buff.sin_f = Sin_out.sin_f;
            Buff.sin_C = Sin_out.sin_C;
            Buff.sin_theta = Sin_out.sin_theta;
            Buff.tt1 = v_time[0];
            UpdateSin=false;
            mtx.unlock();
            UpdateTimes=0;     
// #ifdef DEBUG_PIC

            // Mat Time_Domain = Mat::zeros(315, 800, CV_8UC3);
            // for (int i = 0; i < V_Data_MaxSize - 1; i++) {
            //     if(i < v_size - 1)
            //     line(Time_Domain, Point(i, 314 - abs(v_data[i]) * 40), Point((i + 1), 314 - abs(v_data[(i + 1)]) * 40), Scalar(0, 0, 255), 1, LINE_AA);
            // // line(Time_Domain, Point(i, 314 - abs(v_data[i*2]) * 40), Point((i + 1), 314 - abs(v_data[(i + 1)*2]) * 40), Scalar(0, 0, 255), 1, LINE_AA);
            //     line(Time_Domain, Point(i, 314 - (Sin_out.sin_A * sin(Sin_out.sin_f * 2 * PI *  double(i)/double(fs)*double(num_mean)) + v_mean) * 40),
            //         Point(i+1, 314 - (Sin_out.sin_A* sin(Sin_out.sin_f * 2 * PI * (double(i+1)/double(fs)*double(num_mean))) + v_mean) * 40), Scalar(255, 0, 0), 2, LINE_AA);
            //     line(Time_Domain, Point(i, 314 - (Sin_out.sin_A  * sin(Sin_out.sin_f * 2 * PI *  double(i)/double(fs)*double(num_mean) +Sin_out.sin_theta) + v_mean) * 40),
            //         Point(i+1, 314 - (Sin_out.sin_A  * sin(Sin_out.sin_f * 2 * PI * (double(i+1)/double(fs)*double(num_mean)) + Sin_out.sin_theta) + v_mean) * 40), Scalar(0, 255, 0), 2, LINE_AA);
            // }
            // line(Time_Domain, Point(0, 314 - 0.8 * 40), Point(800, 314 - 0.8 * 40), Scalar(0, 0, 255), 1, LINE_AA);
            // line(Time_Domain, Point(0, 314 - 1.2 * 40), Point(800, 314 - 1.2 * 40), Scalar(0, 0, 255), 1, LINE_AA);
            // line(Time_Domain, Point(0, 314 - 3.0 * 40), Point(800, 314 - 3.0 * 40), Scalar(0, 0, 255), 1, LINE_AA);
            // line(Time_Domain, Point(0, 314 - 2.5 * 40), Point(800, 314 - 2.5 * 40), Scalar(0, 0, 255), 1, LINE_AA);
            // line(Time_Domain, Point(v_size, 314), Point(v_size, 0), Scalar(0, 0, 255), 1, LINE_AA);
            // putText(Time_Domain,"A="+to_string(Sin_out.sin_A)+";  f="+to_string(Sin_out.sin_f)+";  theta="+to_string(Sin_out.sin_theta)+";  C="+to_string(Sin_out.sin_C),
            //         Point2f(5,12),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255,255,0));
            // putText(Time_Domain,"V=A*sin(2 PI f*t+theta)+C",
            //         Point2f(5,30),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255,255,0));
            // imshow("Time_Domain", Time_Domain);
            // Time_Domain.release();
        // keyboard_key = waitKey(WAITKEY_VALUE);
// #endif
        }
        UpdateTimes++;
        vector<double>().swap(v_data);
        vector<struct timeval>().swap(v_time);
        // v_data.shrink_to_fit();
        // v_time.clear();
        // v_time.shrink_to_fit();
        // v_time(vector<double>());
    }
    else 
    {
        mtx.unlock();
    }
    return;

}
/*
void Buff_Class::conv_fft(fftw_complex *Signal_raw_out,int v_data_complex_size,double& conv_value,SinData Sin_in,SinData& Sin_out)
{
    int Nfft = V_Data_MaxSize+V_Data_MaxSize-1;
    fftw_complex* Signal_temp_in = (fftw_complex*) calloc(Nfft, sizeof(fftw_complex));
    fftw_complex* Signal_temp_out = (fftw_complex*) calloc(Nfft, sizeof(fftw_complex));
    if(v_data_complex_size>200){
        for (int i =0;i<V_Data_MaxSize;i++)
            Signal_temp_in[i][0] = Sin_in.sin_A * sin(Sin_in.sin_f * 2 * PI * (double(i)/double(fs)*double(num_mean))) + Sin_in.sin_C;

    }
    else {
        for (int i =0;i<V_Data_MaxSize;i++)
            Signal_temp_in[i][0] = Sin_in.sin_f * 2.0 * PI * Sin_in.sin_A * cos(Sin_in.sin_f * 2.0 * PI * (double(i)/double(fs)*double(num_mean)));

    }
    fftw_plan Signal_temp_Plan;
    Signal_temp_Plan = fftw_plan_dft_1d(Nfft,Signal_temp_in,Signal_temp_out, FFTW_FORWARD, FFTW_ESTIMATE);
    fftw_execute(Signal_temp_Plan);
    fftw_destroy_plan(Signal_temp_Plan);


    fftw_complex *Ifft_in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * Nfft);

    for (int i = 0;i < Nfft;i++) {
//        (a+bi)(c+di)=(ac-bd)+(bc+ad)i
        Ifft_in[i][0] = Signal_raw_out[i][0] * Signal_temp_out[i][0] + Signal_raw_out[i][1] * Signal_temp_out[i][1];
        Ifft_in[i][1] = Signal_raw_out[i][1] * Signal_temp_out[i][0] - Signal_raw_out[i][0] * Signal_temp_out[i][1];
    }

    vector<double> Ifft_out(Nfft);

    fftw_plan Ifft_Plan = fftw_plan_dft_c2r_1d(Nfft,Ifft_in,&Ifft_out[0],FFTW_BACKWARD);
    fftw_execute(Ifft_Plan);
 
    std::vector<double>::iterator biggest = std::max_element(std::begin(Ifft_out), std::end(Ifft_out));
    double biggest_position = std::distance(std::begin(Ifft_out), biggest);
    double conv_biggest =*biggest/Nfft;
    int f_position = 0;
    if(biggest_position >= V_Data_MaxSize)
        f_position = biggest_position - V_Data_MaxSize;
    else
        f_position = biggest_position + V_Data_MaxSize - 1;

//    0 1 2 3 4
//    3 4 0 1 2
    if(v_data_complex_size<=200){
        vector<double> fft_shift(Nfft);
        memcpy(&fft_shift[0], &Ifft_out[V_Data_MaxSize], (V_Data_MaxSize-1)*sizeof(double));
        memcpy(&fft_shift[V_Data_MaxSize-1], &Ifft_out[0], (V_Data_MaxSize)*sizeof(double));

        vector<int> sign;
        double max_value = -1000;
       for(int i = 1;i<Nfft;i++)
       {
           if(max_value<fft_shift[i]/Nfft && i>v_data_complex_size&&i<V_Data_MaxSize && v_data_complex_size <= 300)
           {
               max_value = fft_shift[i]/Nfft;
               conv_biggest = max_value;
            //    max_pt = i;
           }
        //    相邻值做差：
        //     *小于0，赋-1
        //     *大于0，赋1
        //     *等于0，赋0
            
           int diff = fft_shift[i] - fft_shift[i-1];

           if(diff<0)
           {
               sign.push_back(-1);
           }
           else
           {
               sign.push_back(1);
           }
       }
           //再对sign相邻位做差
           //保存极大值和极小值的位置
       int max_confidence = 1;
       for(int i = max_confidence-1;i<(int)sign.size()-max_confidence;i++)//当size==0，size (0)-5会溢出，变成很大的数值， 会存在 i==4，size=0的时候，条件 i<size-5 ==true;
       {

           int diff = 0;
           for(int j = 1;j<max_confidence+1;j++){
               diff += sign[i-j]-sign[i+j];
           }
           if(diff>=2)
           {
    //               if(v_size > 300){
    //                   if (abs(fft_shift[i]/sin_data.Nfft-Ifft_out[biggest_position]/sin_data.Nfft)<0.2){
    //                        f_position = i;
    //                       break;

    //                   }
    //               }
    //               else
    //               {
                    if (abs(fft_shift[i]/Nfft-max_value)<0.2 && i>v_data_complex_size&&i<V_Data_MaxSize){
                        f_position = i;
                    }
    //               }

           }

       }
    }

    if(conv_biggest>conv_value)
    {
//        std::cout << "Max element is " << *bigges<< " at position " << std::distance(std::begin(v), biggest)-N+1 << std::endl;
        conv_value = conv_biggest;
        Sin_out.sin_A = Sin_in.sin_A;
        Sin_out.sin_f = Sin_in.sin_f;
        Sin_out.sin_theta = -2.0*double(f_position-V_Data_MaxSize+1)/double(fs)*double(num_mean);


    }
    fftw_free(Signal_temp_in);
    fftw_free(Signal_temp_out);
    fftw_free(Ifft_in);
    Ifft_out.clear();
    Ifft_out.shrink_to_fit();





}
void Buff_Class::xcorr_fft()
{
    mtx.lock();
    int v_size = Buff.v_data.size();
    if(v_size>0 && Predict_Flag)
    {
        int FN = v_size;
        Predict_Flag = false;
        vector<struct timeval> v_time(Buff.v_time);
        vector<double>v_data(Buff.v_data);

        SinData last_sin;
        last_sin.sin_A = Buff.sin_A;
        last_sin.sin_f = Buff.sin_f;
        last_sin.sin_C = Buff.sin_C;
        last_sin.sin_theta = Buff.sin_theta;

        struct timeval last_sin_time = Buff.tt1;
        mtx.unlock();   
        fftw_complex *fft_in = (fftw_complex*)calloc(FN, sizeof(fftw_complex)); 
        int FILTER_N =15;
        double FILTER_A=1.0;
        for(int i = FILTER_N; i < v_size - 1; i++) {

            if(((fft_in[i - 1][0] - fft_in[i - 2][0]) > FILTER_A) || ((fft_in[i - 2][0] - fft_in[i - 1][0]) > FILTER_A))
            {
            fft_in[i - 1][0] = fft_in[i - 2][0];
            double filter_sum = 0;
            for(int j = i - FILTER_N -1+1; j < i; j++) {
                fft_in[j][0] = fft_in[j + 1][0];
                if(fft_in[j][0]>3.0){
                    filter_sum+=1.5;
                }
               else
                    filter_sum += fft_in[j][0];
            }
                if(filter_sum / (FILTER_N - 1)>3.0)
                    fft_in[i - 1][0]=3.0;
                else if(filter_sum / (FILTER_N - 1)<0.0)
                    fft_in[i - 1][0]=0.0;
                else
                    fft_in[i - 1][0]=filter_sum / (FILTER_N - 1);
                
            }

        }  

        double v_mean =  std::accumulate(std::begin(v_data), std::end(v_data), 0.0) / double(v_size); //均值
        double conv_value=0,best_f=0,best_A=0,best_theta=0;
        double *Signal_raw_in = (double*)calloc(V_Data_MaxSize+V_Data_MaxSize-1, sizeof(double));
        fftw_complex *Signal_raw_out = (fftw_complex*)calloc(V_Data_MaxSize+V_Data_MaxSize-1, sizeof(fftw_complex));
        SinData Sin_out;

        // double min_f = 0.25,max_f=0.45;
        double fft_confidence = 0;
        bool fft_mode=false;
        while((fft_confidence* ((double)fs/(double)num_mean / (double)v_size))<max_f)
        {
            if(fft_confidence* ((double)fs/(double)num_mean / (double)v_size)>min_f){
                fft_mode =true;
                break;
            }
            fft_confidence+=1.0;
        }

        if(fft_mode)
        // if(v_size>=(0 * 2))
        {

        fftw_complex *fft_out = (fftw_complex*)calloc(FN, sizeof(fftw_complex));
        fftw_plan Fft_Plan;
        Fft_Plan = fftw_plan_dft_r2c_1d(FN,&v_data[0],fft_out,FFTW_FORWARD);
        // Fft_Plan = fftw_plan_dft_1d(FN,fft_in,fft_out, FFTW_FORWARD, FFTW_ESTIMATE);
        fftw_execute(Fft_Plan);
        fftw_destroy_plan(Fft_Plan);
//        fftw_free(fft_in);
//        fftw_free(fft_out);
        vector<double>l, h;
        double l_max = 0,h_max=0;

        for (int i = 0; i < FN/2; i++)
        {
//            An=sqrt(a∗a+b∗b)
//            幅度即为A1/N
            // double l_A = 5.0*(double(FN)/1024.0)/(double(v_size)/double(N) *2.0);
            double l_A = 1.0;
//            double l_A = 5.0*256/N;//256 1*5.0  512 2.78 2.5
            l.push_back(l_A * 2.0 * sqrt(fft_out[i][1] / (double)FN * fft_out[i][1] /(double)FN+ fft_out[i][0] / (double)FN * fft_out[i][0] /(double)FN));
            h.push_back((double)i * ((double)fs/(double)num_mean / (double)FN));
            double l_=l_A * 2.0 * sqrt(fft_out[i][1] / (double)FN * fft_out[i][1] /(double)FN+ fft_out[i][0] / (double)FN * fft_out[i][0] /(double)FN);
            double h_ = (double)i * ((double)fs/(double)num_mean / (double)FN);
            if(l_max<l_ && h_>0.2 &&h_<0.40){
                l_max = l_;
                h_max = h_;
            }
//            l.push_back(2.0 * sqrt(x[i].imag() * x[i].imag()+ x[i].real()  * x[i].real())/(double)FN);
//            l.push_back(2.0 * sqrt(x[i].imag / (double)FN * x[i].imag /(double)FN+ x[i].real / (double)FN * x[i].real /(double)FN));
//                    1/T*(0:(L/2))/L

        }

        // double conv_value=0,best_f=0,best_A=0,best_theta=0;
        // fftw_complex *Signal_raw_in = (fftw_complex*)calloc(N+N-1, sizeof(fftw_complex));
        // fftw_complex *Signal_raw_out = (fftw_complex*)calloc(N+N-1, sizeof(fftw_complex));
//        vector<complex_> x(N+N-v_data_complex_size);

        if(v_size>50){
            memcpy(Signal_raw_in, &v_data[0], (v_size)*sizeof(double));

        }else{
            for (int i = 1;i<v_size;i++) {
            Signal_raw_in[i] = v_data[i]-v_data[i-1];
            }
        }

        fftw_plan Signal_Plan = fftw_plan_dft_r2c_1d(V_Data_MaxSize+V_Data_MaxSize-1,Signal_raw_in,Signal_raw_out, FFTW_FORWARD);
        // Signal_Plan = fftw_plan_dft_1d(N+N-1,Signal_raw_in,Signal_raw_out, FFTW_FORWARD, FFTW_ESTIMATE);
        fftw_execute(Signal_Plan);
        fftw_destroy_plan(Signal_Plan);
        fftw_free(Signal_raw_in);
        for (int i = 0; i < FN && h[i] < max_f; i++)
        {
            if (h[i] > min_f && h[i] < max_f)
            {
//                cout<<"sin_A: "<<l[i]*double(v_size)/double(N)<<"    sin_f: "<<h[i]<<"     "<<v_size<<"    "<<double(v_size)/double(N) <<endl;
                double f_condition = min_f;
                if(h[i-1]<=min_f &&h[i+1]>max_f){
                    f_condition = min_f;
                }else {

                    f_condition = h[i];
                }
                for (int k = 0;(f_condition+double(k)*0.01)<h[i+1]&&(f_condition+double(k)*0.01)<max_f;k++) {
                    SinData Sin_in;
                    Sin_in.sin_f=f_condition+double(k)*0.01;
                    Sin_in.sin_A=0;
                    Sin_in.sin_C=v_mean;
                    if(h[i-1]<=min_f && h[i+1]>max_f){
                        if(Sin_in.sin_f<=h[i]){
                            Sin_in.sin_A=(l[i]-l[i-1])/(h[i]-h[i-1])*Sin_in.sin_f+(l[i]/(l[i]-l[i-1])/(h[i]-h[i-1])/h[i]);
                        }else {
                            Sin_in.sin_A=(l[i+1]-l[i])/(h[i+1]-h[i])*Sin_in.sin_f+(l[i]/(l[i+1]-l[i])/(h[i+1]-h[i])/h[i]);
                        }
                    }
                    else if (h[i+1]<=max_f){
                        Sin_in.sin_A=(l[i+1]-l[i])/(h[i+1]-h[i])*Sin_in.sin_f+(l[i]/(l[i+1]-l[i])/(h[i+1]-h[i])/h[i]);
                    }else if(i>0 && h[i-1]>=min_f){
                        Sin_in.sin_A=(l[i]-l[i-1])/(h[i]-h[i-1])*Sin_in.sin_f+(l[i]/(l[i]-l[i-1])/(h[i]-h[i-1])/h[i]);

                    }else {
                        Sin_in.sin_A =0.785;
                    }
                    Sin_in.sin_A=l_max;
                    if(l[i]<0.2)
                        Sin_in.sin_A = 0.785;
                    else if(Sin_in.sin_A<0.695)
                        Sin_in.sin_A=0.695;
                    else if (Sin_in.sin_A>1.435) {
                        Sin_in.sin_A=1.435;
                    }

                    if(v_mean<1.115)
                        v_mean=1.115;
                    else if (v_mean>1.395) {
                        v_mean=1.395;
                    }
                    conv_fft(Signal_raw_out,v_size,conv_value,Sin_in,Sin_out);
                }
            }
        }
        fftw_free(fft_out);
        l.clear();
        l.shrink_to_fit();
        h.clear();
        h.shrink_to_fit();
        }
        else 
        {
            if(v_size>50){
                memcpy(Signal_raw_in, &v_data[0], (v_size)*sizeof(double));
            }
            else
            {
                for (int i = 1;i<v_size;i++) {
                    Signal_raw_in[i] = v_data[i]-v_data[i-1];
                }
            }
            fftw_plan Signal_Plan;
            Signal_Plan = fftw_plan_dft_r2c_1d(V_Data_MaxSize+V_Data_MaxSize-1,Signal_raw_in,Signal_raw_out, FFTW_FORWARD);
            fftw_execute(Signal_Plan);
            fftw_destroy_plan(Signal_Plan);
            fftw_free(Signal_raw_in);

            for (int i = 0; min_f+double(i)*0.01 < max_f; i++)
            {
                SinData Sin_in;
                Sin_in.sin_f=min_f+double(i)*0.01;
                Sin_in.sin_A=0;
                Sin_in.sin_C=v_mean;
                // double sin_f=min_f+double(i)*0.01;
                // double sin_A=0;
                for (int j = 0; 0.695+double(j)*0.05<0.98; j++)
                {
                    Sin_in.sin_A=0.695+j*0.05;
                }
                
                if(v_mean<1.115)
                    v_mean=1.115;
                else if (v_mean>1.395) 
                {
                    v_mean=1.395;
                }

                conv_fft(Signal_raw_out,v_size,conv_value,Sin_in,Sin_out);
            }
        }
        double Best_Conv = 0;
        SinData temp_sin;
        last_sin.sin_C = v_mean;
        conv_fft(Signal_raw_out,v_size,Best_Conv,last_sin,temp_sin);
        double trans_theta = double(((int)(best_theta*10000))%62830)/10000.0;
        if(trans_theta<0.0)
            trans_theta += 2.0*PI;  
        // if(UpdateTimes%20==0 && (v_data[v_data.size()-2]>0.8||(trans_theta< 1.2*PI || trans_theta>1.7*PI)))
        if(UpdateTimes%20==0 && (v_data[v_data.size()-1]>0.8))
        {
            if(v_size<V_Data_MaxSize-50 || (v_size>=V_Data_MaxSize-50&&(trans_theta>1.4*PI || trans_theta < 0.5*PI)))
                UpdateSin = true;

        } 

        // if(UpdateTimes%20==0 && (v_data[v_data.size()-2]>0.8||(trans_theta< 1.5*PI || trans_theta>1.8*PI)) && (v_data[v_data.size()-2]<2.5 )) 
        // if(UpdateTimes%20==0 && (v_data[v_data.size()-2]>1.2||(trans_theta> 0.6*PI && trans_theta<1.5*PI)) && (v_data[v_data.size()-2]<2.5 || (trans_theta<0.4*PI || trans_theta>1.8*PI))) 
        // if(UpdateTimes%20==0) 
        if (best_f>0.22  &&(UpdateSin || (v_size<100 && UpdateTimes%10==0)))
        {
            // cout<<"best_theta"<<double(((int)(best_theta*10000))%62830)/10000.0<<endl;
            if(Best_Conv>conv_value)
            {
                Sin_next.sin_A = last_sin.sin_A;
                Sin_next.sin_f = last_sin.sin_f;
                Sin_next.sin_theta = temp_sin.sin_theta;
                Sin_next.sin_C = last_sin.sin_C;
                sin_time_next=v_time[0];

            }else
            {
                Sin_next.sin_A = Sin_out.sin_A;
                Sin_next.sin_f = Sin_out.sin_f;
                Sin_next.sin_theta = Sin_out.sin_theta;
                Sin_next.sin_C = Sin_out.sin_C;
                sin_time_next=v_time[0];
            }
            UpdateSin = false;

        }
        if(UpdateTimes%10==0 && (v_size<50 && UpdateTimes%10==0)||((v_data[v_data.size()-1]>0.8||(trans_theta> 1.0*PI && trans_theta<2.0*PI))|| (v_size<100))) 
        {
            mtx.lock();
            Buff.sin_A = Sin_next.sin_A;
            Buff.sin_f = Sin_next.sin_f;
            Buff.sin_C = Sin_next.sin_C;
            Buff.sin_theta = Sin_next.sin_theta;
            Buff.tt1 = sin_time_next;
            mtx.unlock();

                        Mat Time_Domain = Mat::zeros(315, 800, CV_8UC3);
            for (int i = 0; i < V_Data_MaxSize - 1; i++) {
                if(i < v_size - 1)
                line(Time_Domain, Point(i, 314 - abs(v_data[i]) * 40), Point((i + 1), 314 - abs(v_data[(i + 1)]) * 40), Scalar(0, 0, 255), 1, LINE_AA);
            // line(Time_Domain, Point(i, 314 - abs(v_data[i*2]) * 40), Point((i + 1), 314 - abs(v_data[(i + 1)*2]) * 40), Scalar(0, 0, 255), 1, LINE_AA);
                line(Time_Domain, Point(i, 314 - (Sin_next.sin_A * sin(Sin_next.sin_f * 2 * PI *  double(i)/double(fs)*double(num_mean)) + v_mean) * 40),
                 Point(i+1, 314 - (Sin_next.sin_A* sin(Sin_next.sin_f * 2 * PI * (double(i+1)/double(fs)*double(num_mean))) + v_mean) * 40), Scalar(255, 0, 0), 2, LINE_AA);
            line(Time_Domain, Point(i, 314 - (Sin_next.sin_A  * sin(Sin_next.sin_f * 2 * PI *  double(i)/double(fs)*double(num_mean) +Sin_next.sin_theta) + v_mean) * 40),
                 Point(i+1, 314 - (Sin_next.sin_A  * sin(Sin_next.sin_f * 2 * PI * (double(i+1)/double(fs)*double(num_mean)) + Sin_next.sin_theta) + v_mean) * 40), Scalar(0, 255, 0), 2, LINE_AA);
            }
            line(Time_Domain, Point(0, 314 - 0.8 * 40), Point(800, 314 - 0.8 * 40), Scalar(0, 0, 255), 1, LINE_AA);
            line(Time_Domain, Point(0, 314 - 1.2 * 40), Point(800, 314 - 1.2 * 40), Scalar(0, 0, 255), 1, LINE_AA);
            line(Time_Domain, Point(0, 314 - 3.0 * 40), Point(800, 314 - 3.0 * 40), Scalar(0, 0, 255), 1, LINE_AA);
            line(Time_Domain, Point(0, 314 - 2.5 * 40), Point(800, 314 - 2.5 * 40), Scalar(0, 0, 255), 1, LINE_AA);
            line(Time_Domain, Point(v_size, 314), Point(v_size, 0), Scalar(0, 0, 255), 1, LINE_AA);
            putText(Time_Domain,"A="+to_string(Sin_next.sin_A)+";  f="+to_string(Sin_next.sin_f)+";  theta="+to_string(Sin_next.sin_theta)+";  C="+to_string(Sin_next.sin_C),
                    Point2f(5,12),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255,255,0));
            putText(Time_Domain,"V=A*sin(2 PI f*t+theta)+C",
                    Point2f(5,30),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255,255,0));
            imshow("Time_Domain", Time_Domain);
            Time_Domain.release();

        }
        fftw_free(fft_in);
        fftw_free(Signal_raw_out);
        v_data.clear();
        v_data.shrink_to_fit();
    }else {
        mtx.unlock();
    }



}
void Buff_Class::xcorr_fft_sub_theta()
{
    mtx.lock();
    int v_size = Buff.v_data.size();
    if(v_size > (0) && Predict_Flag)
    {

        int FN = v_size;
        // if(FN<256)
            // FN = 1024;
        Predict_Flag = false;
//        vector<complex_>v_data_complex(Buff.v_data_complex);
        // vector<long long int> v_time(Buff.v_time);
        // vector<double>v_data(Buff.v_data);
        vector<struct timeval> v_time=Buff.v_time;
        vector<double>v_data=Buff.v_data;

        mtx.unlock();   
        // fftw_complex *fft_in = (fftw_complex*)calloc(FN, sizeof(fftw_complex)); 
        // if (!v_data.empty())
        // {
        //     memcpy(fft_in, &v_data[0], v_data.size()*sizeof(double));
        // }
        int FILTER_N =15;
        double FILTER_A=1.5;
        for(int i = FILTER_N; i < v_size - 1; i++) {

            if(((v_data[i - 1] - v_data[i - 2]) > FILTER_A) || ((v_data[i - 2] - v_data[i - 1]) > FILTER_A))
            {
                //  fft_in[i - 1][0]=fft_in[i - 2][0];
            v_data[i - 1] = v_data[i - 2];
            double filter_sum = 0;
            for(int j = i - FILTER_N -1 +1; j < i; j++) {
                v_data[j] = v_data[j + 1];
                if(v_data[j]>3.0){
                // fft_in[j][0]=0;
                    filter_sum+=1.5;
                }
               else
                    filter_sum += v_data[j];
            }
                if(filter_sum / (FILTER_N - 1)>3.0)
                    v_data[i - 1]=3.0;
                else if(filter_sum / (FILTER_N - 1)<0.0)
                    v_data[i - 1]=0.0;
                else
                    v_data[i - 1]=filter_sum / (FILTER_N - 1);
                
            }

        }  

        double v_mean =  std::accumulate(std::begin(v_data), std::end(v_data), 0.0) / double(v_size); //均值
        double min_offset=1e+5;
        bool had_predicted =false;
        // fftw_complex *Signal_raw_in = (fftw_complex*)calloc(N+N-1, sizeof(fftw_complex));
        // fftw_complex *Signal_raw_out = (fftw_complex*)calloc(N+N-1, sizeof(fftw_complex));
        SinData Sin_out;

        // double min_f = 0.25,max_f=0.45;
        // double min_f = 0.29,max_f=0.325;//0.3  0.31831
        // double min_A = 0.770,max_A=1.055;//0.780  1.045
        double min_C = 2.090 - min_A,max_C = 2.090 - max_A;
        double fft_confidence = 0;
        bool fft_mode=false;
        while((double)fft_confidence * ((double)fs/(double)num_mean / (double)FN)<max_f)
        {
            if((double)fft_confidence * ((double)fs/(double)num_mean / (double)FN)>min_f){
                fft_mode =true;
                break;
            }
            fft_confidence+=1.0;
        }
        if(0)
        // if(v_size>=(0 * 2))
        {
        fftw_complex *fft_out = (fftw_complex*)calloc(FN, sizeof(fftw_complex));
        fftw_plan Fft_Plan;
        Fft_Plan = fftw_plan_dft_r2c_1d(FN,&v_data[0],fft_out, FFTW_FORWARD);
        fftw_execute(Fft_Plan);
        fftw_destroy_plan(Fft_Plan);
//        fftw_free(fft_in);
//        fftw_free(fft_out);
        vector<double>l, h;
        double l_max = 0,h_max=0;

        for (int i = 0; i < FN && (double)i * ((double)fs/(double)num_mean / (double)FN) <= max_f; i++)
        {
//            An=sqrt(a∗a+b∗b)
//            幅度即为A1/N
            // double l_A = 5.0*(double(FN)/1024.0)/(double(v_size)/double(N) *2.0);
            double l_A = 1.0;
//            double l_A = 5.0*256/N;//256 1*5.0  512 2.78 2.5
            l.push_back(l_A * 2.0 * sqrt(fft_out[i][1] / (double)FN * fft_out[i][1] /(double)FN+ fft_out[i][0] / (double)FN * fft_out[i][0] /(double)FN));
            h.push_back((double)i * ((double)fs/(double)num_mean / (double)FN));
            double Curr_l = l_A * 2.0 * sqrt(fft_out[i][1] / (double)FN * fft_out[i][1] /(double)FN+ fft_out[i][0] / (double)FN * fft_out[i][0] /(double)FN);
            double Curr_h = (double)i * ((double)fs/(double)num_mean / (double)FN);
            if(l_max<Curr_l && Curr_h>0.2 &&Curr_h<0.40){
                l_max = Curr_l;
                h_max = Curr_h;
            }

        }

        for (int i = 0; i < FN && h[i] < max_f; i++)
        {
            if (h[i] > min_f && h[i] < max_f)
            {
                double f_condition = min_f;
                if(h[i-1]<=min_f &&h[i+1]>max_f){
                    f_condition = min_f;
                }else {

                    f_condition = h[i];
                }

                for (int k = 0;((f_condition+double(k)*0.01)<h[i+1] && h[i+1] <max_f)||(f_condition+double(k)*0.01)<max_f;k++) {
                    SinData Sin_in;
                    Sin_in.sin_f=f_condition+double(k)*0.01;
                    Sin_in.sin_A=0;
                    if(h[i-1]<=min_f && h[i+1]>max_f){
                        if(Sin_in.sin_f<=h[i]){
                            Sin_in.sin_A=(l[i]-l[i-1])/(h[i]-h[i-1])*Sin_in.sin_f+(l[i]/(l[i]-l[i-1])/(h[i]-h[i-1])/h[i]);
                        }else {
                            Sin_in.sin_A=(l[i+1]-l[i])/(h[i+1]-h[i])*Sin_in.sin_f+(l[i]/(l[i+1]-l[i])/(h[i+1]-h[i])/h[i]);
                        }
                    }
                    else if (h[i+1]<=max_f){
                        Sin_in.sin_A=(l[i+1]-l[i])/(h[i+1]-h[i])*Sin_in.sin_f+(l[i]/(l[i+1]-l[i])/(h[i+1]-h[i])/h[i]);
                    }else if(i>0 && h[i-1]>=min_f){
                        Sin_in.sin_A=(l[i]-l[i-1])/(h[i]-h[i-1])*Sin_in.sin_f+(l[i]/(l[i]-l[i-1])/(h[i]-h[i-1])/h[i]);

                    }else {
                        Sin_in.sin_A =0.785;
                    }
                    Sin_in.sin_A=l_max;
                    if(l[i]<0.2)
                        Sin_in.sin_A = 0.785;
                    else if(Sin_in.sin_A<0.695)
                        Sin_in.sin_A=0.695;
                    else if (Sin_in.sin_A>1.435) {
                        Sin_in.sin_A=1.435;
                    }

                    if(v_mean<1.115)
                        v_mean=1.115;
                    else if (v_mean>1.395) 
                    {
                        v_mean=1.395;
                    }
                    Sin_in.sin_C=v_mean;
                    // conv_fft(Signal_raw_out,v_size/2,conv_value,Sin_in,Sin_out);
                    for(int j = 0;j<360;j++)
                    {
                        double Curr_sub_vla = 0;
                        for (int n = 0; n < v_size && n<=300; n++)//jian shao yun suan liang
                        {
                            double Signal_temp = Sin_in.sin_A * sin(Sin_in.sin_f * 2.0 * PI * (double(n)/double(fs)*double(num_mean))+double(j)*2.0*PI/360.0) + Sin_in.sin_C;
                            Curr_sub_vla+=abs(v_data[n]-Signal_temp);   
                        }    
                        if(min_offset>=Curr_sub_vla)
                        {
                            had_predicted =true;
                            min_offset=Curr_sub_vla;
                            Sin_out.sin_A = Sin_in.sin_A;
                            Sin_out.sin_f = Sin_in.sin_f;
                            Sin_out.sin_C = Sin_in.sin_C;
                            Sin_out.sin_theta = double(j)*2.0*PI/360.0;
                        }
                    }
                }
            }
        }
        fftw_free(fft_out);
        // fftw_free(Signal_raw_out);
        l.clear();
        l.shrink_to_fit();
        h.clear();
        h.shrink_to_fit();
        }else {

            predict_again:
            for (int i = 0; min_f+double(i)*0.005 < max_f; i++)
            {
                SinData Sin_in;
                Sin_in.sin_f=min_f+double(i)*0.005;
                Sin_in.sin_A=0;
                Sin_in.sin_C=v_mean;
                for (int j = 0; min_A+double(j)*0.03<max_A; j++)
                {
                    Sin_in.sin_A=min_A+double(j)*0.03;
                    if(v_mean<min_C)
                        v_mean=min_C;
                    else if (v_mean>1.395) 
                    {
                        v_mean=1.395;
                    }
                    for(int k = 0;k<360;k++)
                    {
                        double Curr_sub_vla = 0;
                        for (int n = 0; n < v_size && n<=300; n++)//jian shao yun suan liang
                        {
                            double Signal_temp = Sin_in.sin_A * sin(Sin_in.sin_f * 2 * PI * (double(n)/double(fs)*double(num_mean))+double(k)*2.0*PI/360.0) + Sin_in.sin_C;
                            Curr_sub_vla+=abs(v_data[n]-Signal_temp);                            
                        }    
                        if(min_offset>=Curr_sub_vla)
                        {
                            min_offset=Curr_sub_vla;
                            Sin_out.sin_A = Sin_in.sin_A;
                            Sin_out.sin_f = Sin_in.sin_f;
                            Sin_out.sin_C = Sin_in.sin_C;
                            Sin_out.sin_theta = double(k)*2.0*PI/360.0;
                        }
                    }
                }

            }
            had_predicted = true;
        }

        if(!had_predicted)
            goto predict_again;
        if(v_size<50)
        {
            if(UpdateTimes%10==0)
                UpdateSin = true;
        }else
        {
            UpdateSin = true;

        }
        if(UpdateSin || (v_size>5 &&v_size<10)){
            mtx.lock();
            Buff.sin_A = Sin_out.sin_A;
            Buff.sin_f = Sin_out.sin_f;
            Buff.sin_C = Sin_out.sin_C;
            Buff.sin_theta = Sin_out.sin_theta;
            Buff.tt1 = v_time[0];
            UpdateSin=false;
            mtx.unlock();
            UpdateTimes=0;     
            // Mat Time_Domain = Mat::zeros(315, 800, CV_8UC3);
            // for (int i = 0; i < N - 1; i++) {
            //     if(i < v_size - 1)
            //     line(Time_Domain, Point(i, 314 - abs(v_data[i]) * 40), Point((i + 1), 314 - abs(v_data[(i + 1)]) * 40), Scalar(0, 0, 255), 1, LINE_AA);
            // // line(Time_Domain, Point(i, 314 - abs(v_data[i*2]) * 40), Point((i + 1), 314 - abs(v_data[(i + 1)*2]) * 40), Scalar(0, 0, 255), 1, LINE_AA);
            //     line(Time_Domain, Point(i, 314 - (Sin_out.sin_A * sin(Sin_out.sin_f * 2 * PI *  double(i)/double(fs)*double(num_mean)) + v_mean) * 40),
            //         Point(i+1, 314 - (Sin_out.sin_A* sin(Sin_out.sin_f * 2 * PI * (double(i+1)/double(fs)*double(num_mean))) + v_mean) * 40), Scalar(255, 0, 0), 2, LINE_AA);
            //     line(Time_Domain, Point(i, 314 - (Sin_out.sin_A  * sin(Sin_out.sin_f * 2 * PI *  double(i)/double(fs)*double(num_mean) +Sin_out.sin_theta) + v_mean) * 40),
            //         Point(i+1, 314 - (Sin_out.sin_A  * sin(Sin_out.sin_f * 2 * PI * (double(i+1)/double(fs)*double(num_mean)) + Sin_out.sin_theta) + v_mean) * 40), Scalar(0, 255, 0), 2, LINE_AA);
            // }
            // line(Time_Domain, Point(0, 314 - 0.8 * 40), Point(800, 314 - 0.8 * 40), Scalar(0, 0, 255), 1, LINE_AA);
            // line(Time_Domain, Point(0, 314 - 1.2 * 40), Point(800, 314 - 1.2 * 40), Scalar(0, 0, 255), 1, LINE_AA);
            // line(Time_Domain, Point(0, 314 - 3.0 * 40), Point(800, 314 - 3.0 * 40), Scalar(0, 0, 255), 1, LINE_AA);
            // line(Time_Domain, Point(0, 314 - 2.5 * 40), Point(800, 314 - 2.5 * 40), Scalar(0, 0, 255), 1, LINE_AA);
            // line(Time_Domain, Point(v_size, 314), Point(v_size, 0), Scalar(0, 0, 255), 1, LINE_AA);
            // putText(Time_Domain,"A="+to_string(Sin_out.sin_A)+";  f="+to_string(Sin_out.sin_f)+";  theta="+to_string(Sin_out.sin_theta)+";  C="+to_string(Sin_out.sin_C),
            //         Point2f(5,12),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255,255,0));
            // putText(Time_Domain,"V=A*sin(2 PI f*t+theta)+C",
            //         Point2f(5,30),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255,255,0));
            // imshow("Time_Domain", Time_Domain);
            // Time_Domain.release();
        }
        UpdateTimes++;

        // v_data(vector<double>());
        // v_data.clear();

        vector<double>().swap(v_data);
        vector<struct timeval>().swap(v_time);
        // v_time.swap(vector<struct timeval>());
        // v_data.shrink_to_fit();
        // v_time.clear();

        // v_time.shrink_to_fit();
        // v_time(vector<double>());




    }
    else 
    {
        mtx.unlock();
    }


}
*/