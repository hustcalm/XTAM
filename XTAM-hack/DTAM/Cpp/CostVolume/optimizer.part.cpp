#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp> 
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>
#include "graphics.hpp"
#include "set_affinity.h"
#include "Cost.hpp"

//relations: 
//gwhatever=0.5*(gwhatever+ghere)
//gright,gdown are negated

using namespace std;
using namespace cv;

typedef size_t st;

//some defines to make typing all this easier
#define down (point+w)
#define up (point-w)
#define right (point+1)
#define left (point-1)
#define here (point)
#define gdown gd[here]
#define gup gu[here]
#define gleft gl[here]
#define gright gr[here]

#define QUIET_DTAM 0
#include "quiet.hpp"

void Cost::initOptimization(){ // Must be thread safe in allocations(i.e. don't do any after first time!)

    int w = cols;
    int h = rows;
    cacheGValues();
    cv::Mat loInd(rows, cols, CV_32SC1);
    cv::Mat loVal(rows, cols, CV_32FC1);

    minv(data, loInd, loVal);
    
    loInd.convertTo(_a, CV_32FC1); // Init a
    assert(aptr == _a.data);
    _a.copyTo(_d);                 // Initialising each element of the primal variable with the data cost minimum

    // Set dual variable q = 0 when optimisation starts
    _qx.create(h, w, CV_32FC1);
    _qx = 0.0;
    _qy.create(h, w, CV_32FC1);
    _qy = 0.0;

    theta = thetaStart;
}

// This function has no equation, I had to derive it from the references
// [3] A first-order primal-dual algorithm for convex problems with applicatioins to imaging
void Cost::computeSigmas(){
    float lambda, alpha, gamma, delta, mu, rho, sigma;
    float L = 4.0; // Lower is better(longer steps), but in theory only >=4 is guaranteed to converge
    lambda = 1.0/theta;
    alpha = epsilon;
    gamma = lambda;
    delta = alpha;
    mu = 2.0*sqrt(gamma*delta)/L;
    
    rho = mu/(2.0*gamma);
    sigma = mu/(2.0*delta);
    sigma_d = rho;
    sigma_q = sigma;
}

void Cost::cacheGValues(){
    int w = cols;
    int h = rows;
    cout<< "Caching G values"<<"\n";
    _gbig.create(h+2, w, CV_32FC1);  // Enough room to safely read off the ends
                                     // Data will be garbage, but we don't care since we don't use it anyway
    _g = Mat(h, w, CV_32FC1, (float*)(_gbig.data)+w); // Begin from the second row since the added surrounding
    
    // Up, Down, Left and Right
    // If we read/write off the end of these something is wrong.
    _gu.create(h, w, CV_32FC1); 
    _gd.create(h, w, CV_32FC1);
    _gl.create(h, w, CV_32FC1);
    _gr.create(h, w, CV_32FC1);
    
    float* b = (float*)(baseImage.data);
    float* g = (float*)(_g.data);
    float* gd = (float*)(_gd.data);
    float* gu = (float*)(_gu.data);
    float* gl = (float*)(_gl.data);
    float* gr = (float*)(_gr.data);

    Mat im_gray;
    cvtColor(baseImage, im_gray, CV_RGB2GRAY);

    // First get derivatives, but use the kind that we need for our purposes, rather than the built in scharr or sobel derivatives. 
    // We will do this by getting the absolute value of the differences to each side of the the current pixel and then taking the max along each direction.
    Mat gx, gy, g1, g2;

    Mat kernel = (Mat_<float>(1,3)<<0.0,-1.0,1.0); // (x, y+1) - (x, y)
    filter2D(im_gray, g1, -1, kernel);
    
    kernel = (Mat_<float>(1,3)<<-1.0,1.0,0.0); // (x, y) - (x, y-1)
    filter2D(im_gray, g2, -1, kernel);

    g1 = abs(g1);
    g2 = abs(g2);
    gx = cv::max(g1, g2);
    
    
    kernel = (Mat_<float>(3,1)<<0.0,-1.0,1.0); // (x+1, y) - (x, y)
    filter2D(im_gray, g1, -1, kernel);

    kernel=(Mat_<float>(3,1)<<-1.0,1.0,0.0); // (x, y) - (x-1, y)
    filter2D(im_gray, g2, -1, kernel);

    g1 = abs(g1);
    g2 = abs(g2);
    gy = cv::max(g1,g2);
    
    // L1 norm
    //_g = gx + gy;

    // L2 norm
    Mat squareX = gx.mul(gx);
    Mat squareY = gy.mul(gy);
    _g = gx + gy;
    sqrt(_g, _g);
    
    toc();
    // The g function (Eq. 5)
    // the paper doesn't specify the values of the exponent or multiplier,
    // so I have chosen them to have a knee at 10% gradient, since this is a 
    // good threshold for edge detectors.
    sqrt(_g, _g); // beta = 1/2
    exp(-3*_g, _g); // alpha = 3

    // Cache interpreted forms of g, for the "matrix" used in
    // section 2.2.3
    st end = w*h;
    for (st point = 0; point < end; point++) { // foreach pixel
        gu[here] = 0.5*(g[up] + g[here]);
        gd[here] = -0.5*(g[down] + g[here]);
        gl[here] = 0.5*(g[left] + g[here]);
        gr[here] = -0.5*(g[right] + g[here]);
    }

    pfShow("g", _g, 0, Vec2d(0,1));
}

static inline float afunc(float* data,float theta,float d,float ds,int a,float lambda){
    return 1.0/(2.0*theta)*ds*ds*(d-a)*(d-a) + data[a]*lambda; // Literal implementation of Eq.14, note the datastep^2 factor to scale correctly
    //return 1.0/(2.0*theta)*(d-a)*(d-a) + data[a]*lambda; // Forget the ds^2 factor for better numerical behavior(sometimes)
    //return std::abs(1.0/(2.0*theta)*ds*ds*(d-a)) + data[a]*lambda; // L1 Version
}


inline float Cost::aBasic(float* data, float l, float ds, float d, float& value){
    int mi = 0;
    float vlast, vnext, v, A, B, C;
    
    float mv = afunc(data, theta, d, ds, 0, lambda);  // a = 0, nemely depth[0]
    v = mv;
    vnext = afunc(data, theta, d, ds, 1, lambda); // a = 1, namely depth[1]
    for(int a = 2; a < l; a++){
        vlast = v;
        v = vnext;
        vnext = afunc(data, theta, d, ds, a, lambda); // depth[a], a belongs to [2, layers]
        if(v < mv){
            A = vlast;
            C = vnext;
            mv = v;
            mi = a - 1; // Index
        }
    }
    
    if(vnext < mv){// Last was best
        value = vnext;
        return (float)l - 1;
    }
    
    if(mi == 0) { // First was best
        value=mv;
        return (float)mi;
    }
    
    B = mv*(1.0 - 1.0e-8); // Avoid divide by zero, since B is already <= others, make < others
    float delt = (A-C)/(A-2*B+C)*.5;
    //value=A/2*(delt)*(delt-1)-B*(delt+1)*(delt-1)+C/2*(delt+1)*(delt);
    value = B - (A-C)*delt/4;
    //B-(A-C)*(A-C)/(A-2*B+C)/8
    //B+(C-A)*delt/2+(A-2*B+C)*delt*delt/2
    
    //cout<<"A"<<A<<" B"<<B<<" C"<<C<<" Ret:"<<delt+float(mi)<<endl;
    //assert(fabs(delt)<=.5);
    return delt + float(mi);
}

static void* Cost_optimizeQD(void* object);
static void* Cost_optimizeA(void* object);
static void launch_optimzer_thread(Cost& cost);

static void launch_optimzer_threads(const Cost* cost){
    pthread_t threadQD, threadA;
    pthread_create(&threadQD, NULL, Cost_optimizeQD, (void*) cost);
    pthread_create(&threadA, NULL, Cost_optimizeA, (void*) cost);
}

static void* Cost_optimizeQD(void* object){
    pthread_setname_np(pthread_self(), "QDthread");
    Cost* cost = (Cost*)object;
    set_affinity(2);
    cost->running_qd = true;

    while(cost->running_a) { // A is not done
        cost->optimizeQD();

        if(allDie) {
            return 0;
        }
    }
    cost->running_qd = false;
}

static void* Cost_optimizeA(void* object){
    pthread_setname_np(pthread_self(), "Athread");
    Cost* cost = (Cost*)object;
    set_affinity(3);
    cost->running_a = true;
    while(cost->running_a) {
        cost->optimizeA();

        if(allDie) {
            return 0;
        }
    }
}

void Cost::optimize(){
    if(!running_a) {
        // Wait for QD to be done
        while(running_qd)
          usleep(100);

        launch_optimzer_threads(this);
    }else{
        cout<<"Already running optimizer!"<<"\n";
    }
}

static void __attribute__ ((noinline)) qcore(
    const float denom,
    st point, 
    const st pstop, 
    float* kx,
    float* ky,
    const float* d,
    const float* gd,
    const float* gu,
    const float* gl,
    const float* gr,
    const float& sigma_q,
    const unsigned& w);

void Cost::optimizeQD(){
    int w = cols;
    int h = rows;
    cout<< "QD optimization run:"<<QDruncount++<<"\n";
    float denom;
    st point; 
    st pstop; 
    float* kx = (float*)(_qx.data);
    float* ky = (float*)(_qy.data);
    float* d = (float*)(_d.data);
    float* a = (float*)(_a.data);

    assert(aptr == _a.data);

    float* g = (float*)(_g.data);
    float* gd = (float*)(_gd.data);
    float* gu = (float*)(_gu.data);
    float* gl = (float*)(_gl.data);
    float* gr = (float*)(_gr.data);

    computeSigmas();
    assert(sigma_q != 0.0);
    assert(sigma_d != 0.0);
    
    // q update ((4 read,1 write)*2 = 8 read, 2 write)
tic();
    denom = 1 + sigma_q*epsilon;
    float nm, pd, kxn, kyn;
    point = 0;
    for(st i = 1; i < h; i++){ // foreach row
        assert(point%w == 0);
        pstop = i*w-1;
        
        qcore(
            denom,
            point, 
            pstop, 
            kx,
            ky,
            d,
            gd,
            gu,
            gl,
            gr,
            sigma_q,
            w);

        point = pstop;
        
        // Last col
        kxn = 0;
        kyn = (ky[here] + sigma_q*((d[here] - d[down])*gdown))/denom; // q(n+1)
        nm = sqrt(kxn*kxn + kyn*kyn); // L2 norm
        pd = max(1.0f, nm);
        kx[here] = 0;
        ky[here] = kyn/pd; // phi function
        point++;
    }

    // Last row
    pstop = h*w - 1;
    for (; point < pstop; point++){
        kxn = (kx[here] + sigma_q*((d[here] - d[right])*gright))/denom;
        kyn = 0;
        nm = sqrt(kxn*kxn+kyn*kyn);
        pd = max(1.0f,nm);
        kx[here] = kxn/pd;
        ky[here] = 0;
    }

    // Last col,row
    kx[here] = 0;
    ky[here] = 0;
toc();

    // d update (10 read,1 write per point)
    denom = 1 + sigma_d/theta;

    // top left
    point = 0;
    pstop = 1;
    d[here] = (d[here] - sigma_d*(gdown*ky[here] + gright*kx[here] - a[here]/theta))/denom;
    point++;

    //toprow
    pstop += w-2;
    for (; point < pstop; point++){
        d[here] = (d[here] - sigma_d*(gdown*ky[here] + gleft*kx[left] + gright*kx[here] - a[here]/theta))/denom;
    }

    // top right
    pstop++;
    d[here] = (d[here] - sigma_d*(gdown*ky[here] + gleft*kx[left] - a[here]/theta))/denom;
    point++;
    assert(point%w == 0);

    // core
    st corestop = w*(h-1);
    for(; pstop < corestop; ) {
        // left core
        pstop++;
        d[here] = (d[here] - sigma_d*(gup*ky[up] + gdown*ky[here] + gright*kx[here] - a[here]/theta))/denom;
        point++;

        // inner core
        pstop += w-2;
        for(; point < pstop; point++){
            d[here] = (d[here] - sigma_d*(gup*ky[up] + gdown*ky[here] + gleft*kx[left] + gright*kx[here] - a[here]/theta))/denom;
        }

        //right core
        pstop++;
        d[here] = (d[here] - sigma_d*(gup*ky[up] + gdown*ky[here] + gleft*kx[left] - a[here]/theta))/denom;
        point++;
    }
    assert(point == pstop);
    assert(pstop == corestop);

    // left bottom
    pstop++;
    d[here] = (d[here] - sigma_d*(gup*ky[up] + gright*kx[here] - a[here]/theta))/denom;
    point++;
        
    // bottom row
    pstop += w-2;
    for(; point < pstop; point++){
        d[here] = (d[here] - sigma_d*(gup*ky[up] + gleft*kx[left] + gright*kx[here] - a[here]/theta))/denom;
    }

    // bottom left
    pstop++;
    d[here] = (d[here] - sigma_d*(gup*ky[up] + gleft*kx[left] - a[here]/theta))/denom;
    point++;

    //debug
    //pfShow("qx",abs(_qx));
    //pfShow("d",_d);
    //pfShow("a",_a);
    
    assert(aptr == _a.data);
    gcheck();
    usleep(1);
}

void Cost::optimizeA(){ 
    assert(aptr == _a.data); // _a is read across threads, so needs to never be de/reallocated
    //usleep(1);
    theta = theta*thetaStep; // update theta

    if (QDruncount > 1000){
        thetaStep = .97;
    }

    if (theta<thetaMin) { // Done optimizing!
        running_a = false;
        gpause();
        //initOptimization();
        stableDepth = _d.clone(); // Always choose more regularized version
        _qx = 0.0;
        _qy = 0.0;
        _d=stableDepth.clone(); // QD might be running, return the depth to it
        theta = thetaStart;
    }

    cout<<"A optimization run: "<<Aruncount++<<endl;
    cout<<"Current Theta: "<<theta<<endl;
    int w = cols;
    int h = rows;
    float* a = (float*)(_a.data);
    float* d = (float*)(_d.data);
    
    //Mat C=dataContainer/hitContainer;
    int l = layers;
    
    float ds = depthStep; 

    
    // a update
    for(st point = 0; point < w*h; point++){ // foreach pixel
        float blank;
        a[point] = aBasic(data + point*l, l, ds, d[point], blank);
    }
    pfShow("d", _d, 0, Vec2d(0, layers));
    pfShow("a", _a, 0, Vec2d(0, layers));

//     //debug: show the energies
//     Mat acost(rows,cols,CV_32FC1);
//     float* ap=acost.ptr<float>(0);
//     for(st point=0;point<w*h;point++){
//         aBasic(data+point*l,l,ds,d[point],ap[point]);
//     }
//     pfShow("A Energy: ", acost,0,Vec2d(0,1e-6));
//     float Ed=sum(acost)[0];
//     float Ee=sum(_qx.mul(_qx)/2*epsilon)[0];
//     
//     cout<<"Data Energy: "<<Ed<<endl;
//     cout<<"Elastic Energy: "<<Ee<<endl;
//     cout<<"Total Energy: "<<Ed+Ee<<endl;
}

static void __attribute__ ((noinline)) qcore(
    const float denom,
    st point, 
    const st pstop, 
    float* kx,
    float* ky,
    const float* d,
    const float* gd,
    const float* gu,
    const float* gl,
    const float* gr,
    const float& sigma_q,
    const unsigned& w){
    float nm,pd,kxn,kyn;
    for (; point < pstop; point++){ // foreach pixel
        // Unnumbered Eq.s at end of section 2.2.3
        // Sign is flipped due to caching a negative value
        //cout<<sigma_q<<endl;
        kxn = (kx[here] + sigma_q*((d[here] - d[right])*gright))/denom;
        kyn = (ky[here] + sigma_q*((d[here] - d[down])*gdown))/denom;
        nm = sqrt(kxn*kxn + kyn*kyn); // L2 norm
        pd = std::max(1.0f,nm);
        kx[here] = kxn/pd;
        ky[here] = kyn/pd;
        //kx[here] = d[here] - d[right];
    }
}
