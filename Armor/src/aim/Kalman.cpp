/*
 *          .,:,,,                                        .::,,,::.
 *        .::::,,;;,                                  .,;;:,,....:i:
 *        :i,.::::,;i:.      ....,,:::::::::,....   .;i:,.  ......;i.
 *        :;..:::;::::i;,,:::;:,,,,,,,,,,..,.,,:::iri:. .,:irsr:,.;i.
 *        ;;..,::::;;;;ri,,,.                    ..,,:;s1s1ssrr;,.;r,
 *        :;. ,::;ii;:,     . ...................     .;iirri;;;,,;i,
 *        ,i. .;ri:.   ... ............................  .,,:;:,,,;i:
 *        :s,.;r:... ....................................... .::;::s;
 *        ,1r::. .............,,,.,,:,,........................,;iir;
 *        ,s;...........     ..::.,;:,,.          ...............,;1s
 *       :i,..,.              .,:,,::,.          .......... .......;1,
 *      ir,....:rrssr;:,       ,,.,::.     .r5S9989398G95hr;. ....,.:s,
 *     ;r,..,s9855513XHAG3i   .,,,,,,,.  ,S931,.,,.;s;s&BHHA8s.,..,..:r:
 *    :r;..rGGh,  :SAG;;G@BS:.,,,,,,,,,.r83:      hHH1sXMBHHHM3..,,,,.ir.
 *   ,si,.1GS,   sBMAAX&MBMB5,,,,,,:,,.:&8       3@HXHBMBHBBH#X,.,,,,,,rr
 *   ;1:,,SH:   .A@&&B#&8H#BS,,,,,,,,,.,5XS,     3@MHABM&59M#As..,,,,:,is,
 *  .rr,,,;9&1   hBHHBB&8AMGr,,,,,,,,,,,:h&&9s;   r9&BMHBHMB9:  . .,,,,;ri.
 *  :1:....:5&XSi;r8BMBHHA9r:,......,,,,:ii19GG88899XHHH&GSr.      ...,:rs.
 *  ;s.     .:sS8G8GG889hi.        ....,,:;:,.:irssrriii:,.        ...,,i1,
 *  ;1,         ..,....,,isssi;,        .,,.                      ....,.i1,
 *  ;h:               i9HHBMBBHAX9:         .                     ...,,,rs,
 *  ,1i..            :A#MBBBBMHB##s                             ....,,,;si.
 *  .r1,..        ,..;3BMBBBHBB#Bh.     ..                    ....,,,,,i1;
 *   :h;..       .,..;,1XBMMMMBXs,.,, .. :: ,.               ....,,,,,,ss.
 *    ih: ..    .;;;, ;;:s58A3i,..    ,. ,.:,,.             ...,,,,,:,s1,
 *    .s1,....   .,;sh,  ,iSAXs;.    ,.  ,,.i85            ...,,,,,,:i1;
 *     .rh: ...     rXG9XBBM#M#MHAX3hss13&&HHXr         .....,,,,,,,ih;
 *      .s5: .....    i598X&&A&AAAAAA&XG851r:       ........,,,,:,,sh;
 *      . ihr, ...  .         ..                    ........,,,,,;11:.
 *         ,s1i. ...  ..,,,..,,,.,,.,,.,..       ........,,.,,.;s5i.
 *          .:s1r,......................       ..............;shs,
 *          . .:shr:.  ....                 ..............,ishs.
 *              .,issr;,... ...........................,is1s;.
 *                 .,is1si;:,....................,:;ir1sr;,
 *                    ..:isssssrrii;::::::;;iirsssssr;:..
 *                         .,::iiirsssssssssrri;;:.
 */

#include <iostream>
#include <stdexcept>
#include <math.h>
#include "Kalman.h"
#include "SerialPort.h"

using namespace std;

Kalman::Kalman()
{
    Xt_hat.setZero(); Xt_now.setZero(); Xt_pre.setZero();
    Pt_hat.setZero(); Pt_now.setZero(); Pt_pre.setZero();

    Zt.setZero();
    Kt.setOnes();
    H << 1, 0;

    I.setIdentity();
}

void Kalman::time_init(double dt)
{
    Fk <<   1,  -1 * dt,
            0,   1;

    Bk <<   dt * dt / 2, dt;

    this->dt = dt;

}

//估计方程
void Kalman::Estimation(double at, double v, double angle_solution)
{
    //先验估计f
    Xt_now = Fk * Xt_pre + Bk * at;

    angle.push_back(angle_solution);
    speed.push_back(v);
    if (angle.size() > ONE && speed.size() > ONE)
    {
        angle.erase(angle.begin());
        speed.erase(speed.begin());

        Zt << angle[0], speed[0];

        //先验估计
        //        Pt_now = Fk * Pt_pre * Fk.transpose() + Q;
        Pt_now(0, 0) = Pt_pre(0, 0) + Q(0, 0) - (Pt_pre(0, 0) - Pt_pre(1, 0)) * dt + Pt_pre(1, 1) * dt * dt;
        Pt_now(0, 1) = Pt_pre(0, 1) - Pt_pre(1, 1) * dt;
        Pt_now(1, 0) = Pt_pre(1, 0) - Pt_pre(1, 1) * dt;
        Pt_now(1, 1) = Pt_pre(1, 0) + Q(1, 1);

        //卡尔曼
        Eigen::Vector2d k1 = Pt_now * H.transpose();
        Eigen::Matrix<double, 1, 1> k2 = H * Pt_now * H.transpose() + R;
        Kt = k1 * k2.inverse();

        //最优估计解
        Xt_hat = Xt_now + Kt * (Zt(0, 0) - H * Xt_now);

        //后验估计
        Pt_hat = (I - Kt * H) * Pt_now;

    }
}

void Kalman::End()
{
    Xt_now = Xt_hat;
    Xt_pre = Xt_now;

    Pt_pre = Pt_hat;
    Pt_now.setZero();
}
