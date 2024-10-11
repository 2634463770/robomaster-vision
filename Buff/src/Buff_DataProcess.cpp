#include "Buff_Task.h"
#include "tinystr.h"
#include "tinyxml.h"
// int V_Data_MaxSize = 512;
// // int num_mean = num_mean;
// // double fs =110;
// double interval_f = 0.005;
// double interval_A = 0.03;
// int num_mean = 2;

int UpdateTimes = 0;//更新次数
bool Predict_Flag = false;//预测标志
vector<int>angle_sum;//记录角度方向，size为5
vector<double> dAngle_fft;//每两帧记录一次角度差;size为2
int record_num = 0;//每num_mean记录一次  数据次数 ，记录后清零
// float slow_num = -1;
// vector<float> V_sin;//角度 也是25个时间点 size=25    as 0-12   13  18点
mutex mtx;

void Buff_Class::update_Vdata()
{
    if(Buff.v_data.size()==0 && record_num==0){ //第一次识别
        mtx.lock();
        Buff.Last_angle = Buff.angle;
        Buff.Last_pts = Buff.inaction.center;
//        dAngle.push_back((360 * n + Buff.angle) - Buff.Last_angle_);
        if(shootmode==1)
        {
            Buff.v_data.push_back(0);
            Buff.v_time.push_back(Buff.ThisFrameTime);        
        }
        Buff.last_v = 0;
        mtx.unlock();
        record_num = 1;
        return;
    }
    //两帧间角度差大于15度，则圈数变化  避免旋转时 角度为0 突变成 -359，调节角度平缓过度
    if (Buff.Last_angle - (360 * Buff.rotations + Buff.angle) > 15) {
        Buff.rotations = Buff.rotations + 1;

    }
    if (Buff.Last_angle - (360.0 * Buff.rotations + Buff.angle) < -15.0) {

        Buff.rotations = Buff.rotations - 1;
    }
    double max_length = max(Buff.inaction.size.width,Buff.inaction.size.height);
    //距离差距大，则认为是扇叶变化了
    if (fabs(Buff.Last_pts.x - Buff.inaction.center.x) > max_length || 
        fabs(Buff.Last_pts.y - Buff.inaction.center.y) > max_length)
    {

        Buff.rotations = 0;
    }
    //角度差距小
    if (abs((360 * Buff.rotations + Buff.angle - Buff.Last_angle)) < 25.0)
    {
        dAngle_fft.push_back((360 * Buff.rotations + Buff.angle) - Buff.Last_angle);
        if (record_num % num_mean == 0) //当record_num==num_mean 记录速度数据
        {
            //记录角度方向，size为5
            if ((360 * Buff.rotations + Buff.angle - Buff.Last_angle) < 0.0f)
                angle_sum.push_back(-1);
            else
                angle_sum.push_back(1);

            if (angle_sum.size() >= 6)
            {
                int sum_V = angle_sum[0] + angle_sum[1] + angle_sum[2] + angle_sum[3] + angle_sum[4] + angle_sum[5];
                extern int V_direction ;
                if (sum_V < -4)
                {
                    V_direction = 1;
                }
                if (sum_V > 4)
                {
                    V_direction = -1;
                }
                angle_sum.clear();
                angle_sum.shrink_to_fit();
            }
            double dAngle_sum = 0;
            for (int j=0;j < num_mean;j++) {
                dAngle_sum = dAngle_sum + dAngle_fft[j]; //num_mean帧的速度
            }
            if(shootmode==1)
            {
                mtx.lock();
                int size_reduce = 0;
                Buff.v_time.push_back(Buff.ThisFrameTime);
                Buff.v_data.push_back(abs(dAngle_sum/180.0*PI*fs/num_mean));//计算每num_mean帧的平均速度
                if(Buff.v_data.size()>V_Data_MaxSize-size_reduce) //控制数据长度 V_Data_MaxSize-size_reduce
                {
                    Buff.v_time.erase(Buff.v_time.begin(), Buff.v_time.begin() + Buff.v_time.size() - (V_Data_MaxSize-size_reduce));
                    Buff.v_data.erase(Buff.v_data.begin(), Buff.v_data.begin() + Buff.v_data.size() - (V_Data_MaxSize-size_reduce));
                    Buff.v_time.shrink_to_fit();
                    Buff.v_data.shrink_to_fit();
                }
                Predict_Flag =true;
                UpdateTimes++;
            }
            mtx.unlock();
            dAngle_fft.clear();
            dAngle_fft.shrink_to_fit();
            record_num = 0;
        }

    }
    record_num++;//当record_num==num_mean 记录速度数据
    return;

}
void Buff_Class::clear_data(){//清楚数据
    if(!Buff.clear_flag){
        mtx.lock();
        Buff.init_flag = false;
        Predict_Flag = false;
        angle_sum.clear();//记录角度方向，size为5
        angle_sum.shrink_to_fit();
        dAngle_fft.clear();//每两帧记录一次角度差;size为2
        dAngle_fft.shrink_to_fit();
        record_num = 0;
        Buff.rotations = 0;
        // slow_num =-1;
        // V_sin.clear();
        // V_sin.shrink_to_fit();
        Buff.first_find =false;
        Buff.action.clear();
        Buff.action.shrink_to_fit();
        Buff.action_fan.clear();
        Buff.action_fan.shrink_to_fit();
        Buff.v_time.clear();
        Buff.v_time.shrink_to_fit();
        Buff.v_data.clear();
        Buff.v_data.shrink_to_fit();
        Buff.pts.clear();
        Buff.pts.shrink_to_fit();
        Buff.sin_A=0;
        Buff.radius=0;//buff半径
        Buff.clear_flag = true;
        Buff.pnp_time = getCurrentTime();
        mtx.unlock();


    }
    return;
}
void Buff_Class::Buff_init(){
    if(!Buff.init_flag){
        Buff.init_flag = true;
        Buff.pnp_time = getCurrentTime();

    }
    return;
}
//XML 优化预测的速度，但是没有使用，看不懂可以不管

bool Buff_Class::MakeDataXML(string FilePath)
{
    TiXmlDocument xmlDocument;
    // 添加XML声明
    xmlDocument.LinkEndChild(new TiXmlDeclaration("1.0", "UTF-8", ""));

    // 添加根元素
    TiXmlElement* xmlRoot = new TiXmlElement("SinData");
    xmlDocument.LinkEndChild(xmlRoot);
    TiXmlElement* xmlChild_DataInfo = new TiXmlElement("DataInfo");
    xmlRoot->LinkEndChild(xmlChild_DataInfo);
    TiXmlElement* xmlChild_Frequency_Size = new TiXmlElement("Frequency_Size");
    xmlChild_DataInfo->LinkEndChild(xmlChild_Frequency_Size);
    TiXmlElement* xmlChild_Amplitude_Size = new TiXmlElement("Amplitude_Size");
    xmlChild_DataInfo->LinkEndChild(xmlChild_Amplitude_Size);
    TiXmlElement* xmlChild_theta_Size = new TiXmlElement("Theta_Size");
    xmlChild_DataInfo->LinkEndChild(xmlChild_theta_Size);
    TiXmlElement* xmlChild_V_Data_MaxSize = new TiXmlElement("V_Data_MaxSize");
    xmlChild_DataInfo->LinkEndChild(xmlChild_V_Data_MaxSize);
    TiXmlElement* xmlChild_num_mean = new TiXmlElement("num_mean");
    xmlChild_DataInfo->LinkEndChild(xmlChild_num_mean);
    TiXmlElement* xmlChild_fs = new TiXmlElement("fs");
    xmlChild_DataInfo->LinkEndChild(xmlChild_fs);

    TiXmlElement* xmlChild_min_f = new TiXmlElement("min_f");
    xmlChild_DataInfo->LinkEndChild(xmlChild_min_f);
    TiXmlElement* xmlChild_max_f = new TiXmlElement("max_f");
    xmlChild_DataInfo->LinkEndChild(xmlChild_max_f);
    TiXmlElement* xmlChild_min_A = new TiXmlElement("min_A");
    xmlChild_DataInfo->LinkEndChild(xmlChild_min_A);
    TiXmlElement* xmlChild_max_A = new TiXmlElement("max_A");
    xmlChild_DataInfo->LinkEndChild(xmlChild_max_A);
    TiXmlElement* xmlChild_interval_f = new TiXmlElement("interval_f");
    xmlChild_DataInfo->LinkEndChild(xmlChild_interval_f);
    TiXmlElement* xmlChild_interval_A = new TiXmlElement("interval_A");
    xmlChild_DataInfo->LinkEndChild(xmlChild_interval_A);



    //根元素下添加子元素1
    // int datanum=0;

    int Frequency_Size = 0;
    int Amplitude_Size = 0;


    for (int i = 0; min_f+double(i)*interval_f < max_f; i++,Frequency_Size++)
    {
        TiXmlElement* xmlChild_Frequency = new TiXmlElement("Frequency");
        xmlRoot->LinkEndChild(xmlChild_Frequency);
        xmlChild_Frequency->SetAttribute("f", to_string(min_f+double(i)*interval_f).c_str());//设置属性
        double Frequency_Curr = min_f+double(i)*interval_f;
        Amplitude_Size=0;
        for (int j = 0; min_A+double(j)*interval_A<max_A; j++,Amplitude_Size++)
        {
            double Amplitude_Curr = min_A+double(j)*interval_A;
            TiXmlElement* xmlChild_Amplitude = new TiXmlElement("Amplitude");
            xmlChild_Amplitude->SetAttribute("A", to_string(min_A+double(j)*interval_A).c_str());//设置属性
            xmlChild_Frequency->LinkEndChild(xmlChild_Amplitude);
            for(int k = 0;k<Theta_Size;k++)
            {
                TiXmlElement* xmlChild_theta = new TiXmlElement("theta");
                xmlChild_theta->SetAttribute("theta", to_string(double(k)*2.0*PI/double(Theta_Size)).c_str());//设置属性
                string SinData="";
                for (int n = 0; n < V_Data_MaxSize; n++)
                {
                    SinData = SinData + to_string(Amplitude_Curr * sin(Frequency_Curr * 2.0 * PI * (double(n)/double(fs)*double(num_mean))+
                    double(k)*2.0*PI/double(Theta_Size))) + " ";
                    // datanum++;
                }
                xmlChild_theta->LinkEndChild(new TiXmlText(SinData.c_str()));
                xmlChild_Amplitude->LinkEndChild(xmlChild_theta);
            }
        }
    }
    xmlChild_Frequency_Size->LinkEndChild(new TiXmlText(to_string(Frequency_Size).c_str()));
    xmlChild_Amplitude_Size->LinkEndChild(new TiXmlText(to_string(Amplitude_Size).c_str()));
    xmlChild_theta_Size->LinkEndChild(new TiXmlText(to_string(Theta_Size).c_str()));
    xmlChild_V_Data_MaxSize->LinkEndChild(new TiXmlText(to_string(V_Data_MaxSize).c_str()));
    xmlChild_num_mean->LinkEndChild(new TiXmlText(to_string(num_mean).c_str()));
    xmlChild_fs->LinkEndChild(new TiXmlText(to_string(fs).c_str()));
    xmlChild_min_f->LinkEndChild(new TiXmlText(to_string(min_f).c_str()));
    xmlChild_max_f->LinkEndChild(new TiXmlText(to_string(max_f).c_str()));
    xmlChild_min_A->LinkEndChild(new TiXmlText(to_string(min_A).c_str()));
    xmlChild_max_A->LinkEndChild(new TiXmlText(to_string(max_A).c_str()));
    xmlChild_interval_A->LinkEndChild(new TiXmlText(to_string(interval_A).c_str()));
    xmlChild_interval_f->LinkEndChild(new TiXmlText(to_string(interval_f).c_str()));
    //保存xml文件    
    return xmlDocument.SaveFile(FilePath.c_str());
}
double**** Buff_Class::LoadDataXML(string FilePath)
{
    TiXmlDocument doc(FilePath.c_str());
    bool loadOkay = doc.LoadFile();
    if (!loadOkay) {
        printf( "Could not load test file %s. Error='%s'. Exiting.\n", FilePath.c_str(),doc.ErrorDesc() );
        return NULL;
    }

    // get dom root of 'phonebookdata.xml', here root should be 'phonebook'.
    TiXmlElement* root = doc.RootElement();
    TiXmlNode * DataInfoNode = root->FirstChild("DataInfo");
    if (atof(DataInfoNode->FirstChild("num_mean")->ToElement()->GetText()) != num_mean || 
        atof(DataInfoNode->FirstChild("fs")->ToElement()->GetText()) != fs ||
        atof(DataInfoNode->FirstChild("Theta_Size")->ToElement()->GetText()) != Theta_Size ||
        atof(DataInfoNode->FirstChild("V_Data_MaxSize")->ToElement()->GetText()) != V_Data_MaxSize ||
        atof(DataInfoNode->FirstChild("interval_f")->ToElement()->GetText()) != interval_f ||
        atof(DataInfoNode->FirstChild("interval_A")->ToElement()->GetText()) != interval_A ||
        atof(DataInfoNode->FirstChild("max_f")->ToElement()->GetText()) != max_f ||
        atof(DataInfoNode->FirstChild("max_A")->ToElement()->GetText()) != max_A ||
        atof(DataInfoNode->FirstChild("min_f")->ToElement()->GetText()) != min_f ||
        atof(DataInfoNode->FirstChild("min_A")->ToElement()->GetText()) != min_A
    )
        return NULL;

    int Frequency_Size=atof(DataInfoNode->FirstChild("Frequency_Size")->ToElement()->GetText());
    int Amplitude_Size=atof(DataInfoNode->FirstChild("Amplitude_Size")->ToElement()->GetText());
    int theta_Size=atof(DataInfoNode->FirstChild("Theta_Size")->ToElement()->GetText());
    double**** SinDataSheet=(double****)malloc(Frequency_Size*sizeof (double));
    for (int i = 0;i<Frequency_Size;i++) {
        SinDataSheet[i] = (double***)malloc(Amplitude_Size*sizeof (double));
        for (int j = 0;j<Amplitude_Size;j++) {
            SinDataSheet[i][j] = (double**)malloc(theta_Size*sizeof (double));
            for (int k = 0;k<theta_Size;k++) {
                SinDataSheet[i][j][k] = (double*)malloc(V_Data_MaxSize*sizeof (double));
            }
        }
    }
    struct SinData_lable
    {
        double Frequency;
        double Amplitude;
        double theta;
    };
    SinData_lable*** SinData_lable_Sheet=(SinData_lable***)malloc(Frequency_Size*sizeof (SinData_lable));
    for (int i = 0;i<Frequency_Size;i++) {
        SinData_lable_Sheet[i] = (SinData_lable**)malloc(Amplitude_Size*sizeof (SinData_lable));
        for (int j = 0;j<Amplitude_Size;j++) {
            SinData_lable_Sheet[i][j] = (SinData_lable*)malloc(theta_Size*sizeof (SinData_lable));
        }
    }
    unsigned int i =0;
    int datanum=0;
    for(TiXmlNode*  Frequency_item = root->FirstChild( "Frequency" );
             Frequency_item;
             Frequency_item = Frequency_item->NextSibling( "Frequency" ) ,i++) {
        double Frequency_Curr=atof(Frequency_item->ToElement()->FirstAttribute()->Value());
        unsigned int j =0;
        for( TiXmlNode*  Amplitude_item = Frequency_item->FirstChild( "Amplitude" );
                 Amplitude_item;
                 Amplitude_item = Amplitude_item->NextSibling( "Amplitude" ),j++) {
            double Amplitude_Curr=atof(Amplitude_item->ToElement()->FirstAttribute()->Value());
            int k = 0;
            for( TiXmlNode*  theta_item = Amplitude_item->FirstChild( "theta" );
                     theta_item;
                     theta_item = theta_item->NextSibling( "theta" ),k++) {
                double theta_Curr=atof(theta_item->ToElement()->FirstAttribute()->Value());
                SinData_lable_Sheet[i][j][k]=SinData_lable{Frequency_Curr,Amplitude_Curr,theta_Curr};
                const char* V_Data = theta_item->ToElement()->GetText();
                if (V_Data) {
                    char *p_V_Data = const_cast<char *>(V_Data);
                    char * p = strtok(p_V_Data, " ");
                    int Curr_num=0;
                    while (p != NULL)
                    {
                        SinDataSheet[i][j][k][Curr_num]=atof(p);
                        Curr_num++;
                        datanum++;
                        p = strtok(NULL, " ");
                    }
                }
            }
        }
    }

    return SinDataSheet;
}