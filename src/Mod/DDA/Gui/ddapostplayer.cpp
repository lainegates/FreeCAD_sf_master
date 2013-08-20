#include "ddapostplayer.h"
#include "ui_ddapostplayer.h"
#include <qtcore/QQueue>
#include <qtcore/QSemaphore>
#include <QtCore/QEventLoop>
#include <QtCore/QMutexLocker>
#include <QtGui/QMouseEvent>
#include <QtCore/QPoint>
#include <QtCore/qtimer.h>
//#include <crtdbg.h>
#include <direct.h>

namespace DDAGui{
    const int MAXFRAMESNUM = 3;
    const double TIMEINTERVAL = 200;  // milliseconds time interval between every two frames
    int playSpeed = 1;

    class SynchronizationQueue
    {
    public:
        void enqueue(FrameData& frameData){
            _lock.lock();
            _queue.enqueue(frameData);
            _lock.unlock();
        }
        FrameData dequeue(){
            QMutexLocker locker(&_lock);
            return _queue.dequeue();
        }
        FrameData& head(){
            QMutexLocker locker(&_lock);
            return _queue.head();
        }

        int size(){
            QMutexLocker locker(&_lock);
            return _queue.size();
        }
        void clear(){
            QMutexLocker locker(&_lock);
            _queue.clear();
        }

    
    private:
        QQueue<FrameData> _queue;
        QMutex _lock;

    }frameQueue;
    
}

using namespace DDAGui;
//////////////////////////////////////
////  producer
//////////////////////////////////////
void FrameProducer::_realProduce1Frame()
{
    if(_dataTaker->hasNextFrame())
    {
        if(!_lock.tryLock() ) // wait for 0.5 second to acquire free resource
            return;
        if( frameQueue.size()>=MAXFRAMESNUM)
        {
            _lock.unlock();
            return;
        }

#ifdef DEBUGPRODUCER
        msg("get new free resource.\n");
#endif
        frameQueue.enqueue(_dataTaker->getNextGraph());
#ifdef DEBUGPRODUCER
        msg("one frame parsed done.\n");
#endif
        _lock.unlock();
        emit oneFrameProduced();

    }
    else
        emit noMoreFrames();
}

void FrameProducer::produceFrames()
{
    _realProduce1Frame();
    if( frameQueue.size()<MAXFRAMESNUM)  // make sure more than frames are available
        _realProduce1Frame();
}

void FrameProducer::produce1Frame()
{
    _realProduce1Frame();
    msg("+++++++++++++++1 frame produced==========\n");
}

void FrameProducer::setStartFrameIndex(int frameIndex)
{
#ifdef DEBUGPRODUCER
    char tmpStr[200];
    sprintf(tmpStr , "set start frame %d\n" , frameIndex);
    msg(tmpStr);
#endif

    _lock.lock();
    _dataTaker->setCurrentFrameIdx(frameIndex);
    _lock.unlock();
}



//////////////////////////////////////
////  consumer
//////////////////////////////////////


void FrameConsumer::pausePlay()
{
#ifdef DEBUGCONSUMER
    msg("================pause play=====================\n");
#endif
    _ifContinuePlay = false;
}


void FrameConsumer::acceptPlayEnd()
{
#ifdef DEBUGCONSUMER
    msg("================no more frames.=====================\n");
#endif
    _ifProduceDone = true;
}

void FrameConsumer::stopPlay()
{
#ifdef DEBUGCONSUMER
    msg("================stop play=====================\n");
#endif
    _ifContinuePlay = false;
    emit frameIndexChanged(0);
}

void FrameConsumer::startPlay()
{
#ifdef DEBUGCONSUMER
    msg("================start play.=====================\n");
#endif
    _ifProduceDone = false;
    _ifContinuePlay = true;
    emit requireFrames();
}


void FrameConsumer::display1Frame(int frameIdx)
{
    //char tmpStr[200];
    //sprintf(tmpStr , "current frame %d\n" , frameIdx);
    //msg(tmpStr);

    _ifContinuePlay = false;
    emit setStartFrameIndex(frameIdx);
    emit require1Frame();
}

void FrameConsumer::displayFrames()
{
    msg("start display\n");
    if(!_lock.tryLock() )
        return;

    if(frameQueue.size()<1)
    {
        _lock.unlock();
        return;
    }

    assert(frameQueue.size()>0);
//     Sleep(TIMEINTERVAL);

    int FrameIdx = frameQueue.head().frameNo;
    _frameMaker->updateGraph(frameQueue.dequeue());
    emit frameIndexChanged(FrameIdx);

    if(_ifContinuePlay){
        if(!_ifProduceDone)
            emit requireFrames();
    }
    else
        frameQueue.clear();

    if(!_ifProduceDone)   // player already finished, now play the last rest frames.
    {
        int size = frameQueue.size();
        while(size--)
        {
            int FrameIdx = frameQueue.head().frameNo;
            _frameMaker->updateGraph(frameQueue.dequeue());
            emit frameIndexChanged(FrameIdx);
        }
    }
    _lock.unlock();
}

//////////////////////////////////////
////  DDAPostplayer
//////////////////////////////////////
DDAPostPlayer::DDAPostPlayer(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DDAPostPlayer)
{
    ui->setupUi(this);
//     ui->playSlider->installEventFilter(this);

    _producer = new FrameProducer;
    _consumer = new FrameConsumer;
    _produceThread = new QThread;
    _consumeThread = new QThread;

//     _producer->moveToThread(_produceThread);
//     _consumer->moveToThread(_consumeThread);
//     _consumeThread->start();
//     _produceThread->start();
// 
    initConnection();
}

DDAPostPlayer::~DDAPostPlayer()
{
    delete _producer;
    delete _consumer;
    _produceThread->terminate();
    _consumeThread->terminate();
    _produceThread->wait();
    _consumeThread->wait();
    delete _produceThread;
    delete _consumeThread;
    delete ui;
}

void DDAPostPlayer::initPlay(int framesNum)
{
    ui->frameNoSpinBox->setRange(0 , framesNum-1);
    ui->frameNoSpinBox->setValue(0);
    ui->playSlider->setRange(0 , framesNum-1);
    ui->playSlider->setValue(0);
}

void DDAPostPlayer::initConnection()
{
    QObject::connect(_producer , SIGNAL(noMoreFrames()) , _consumer , SLOT(acceptPlayEnd()) , Qt::QueuedConnection);
    QObject::connect(_producer , SIGNAL(oneFrameProduced()) , _consumer , SLOT(displayFrames()) , Qt::QueuedConnection);
    
    QObject::connect(_consumer , SIGNAL(frameIndexChanged(int)) , this , SLOT(frameIndexChanged(int)) , Qt::QueuedConnection);
    QObject::connect(_consumer , SIGNAL(setStartFrameIndex(int)) , _producer , SLOT(setStartFrameIndex(int)) , Qt::QueuedConnection);
    QObject::connect(_consumer , SIGNAL(requireFrames()) , _producer , SLOT(produceFrames()) , Qt::QueuedConnection);
    QObject::connect(_consumer , SIGNAL(require1Frame()) , _producer , SLOT(produce1Frame()) , Qt::QueuedConnection);
    
    QObject::connect(ui->openBtn , SIGNAL(pressed()) , this , SLOT(openBtnPressed()));
    QObject::connect(ui->playBtn , SIGNAL(pressed()) , this , SLOT(playBtnPressed()));
    QObject::connect(ui->stopBtn , SIGNAL(pressed()) , this , SLOT(stopBtnPressed()));
    QObject::connect(ui->frameNoSpinBox , SIGNAL(valueChanged(int)) , this , SLOT(spinBoxValueChanged(int)));
    QObject::connect(ui->frameNoSpinBox , SIGNAL(valueChanged(int)) , ui->playSlider , SLOT(setValue(int)));
    QObject::connect(ui->playSlider , SIGNAL(valueChanged(int)) , this , SLOT(sliderValueChanged(int)));
    QObject::connect(ui->playSlider , SIGNAL(valueChanged(int)) , ui->frameNoSpinBox , SLOT(setValue(int)));

    QObject::connect(this , SIGNAL(startPlay()) , _consumer , SLOT(startPlay()) , Qt::QueuedConnection);
    QObject::connect(this , SIGNAL(play1Frame(int)) , _consumer , SLOT(display1Frame(int)) , Qt::QueuedConnection);
    QObject::connect(this , SIGNAL(stopPlay()) , _consumer , SLOT(stopPlay()) , Qt::QueuedConnection);
    QObject::connect(this , SIGNAL(pausePlay()) , _consumer , SLOT(pausePlay()) , Qt::QueuedConnection);

     //QObject::connect(_producer , SIGNAL(noMoreFrames()) , _consumer , SLOT(acceptPlayEnd()));
     //QObject::connect(_producer , SIGNAL(oneFrameProduced()) , _consumer , SLOT(displayFrames()));
 
     //QObject::connect(_consumer , SIGNAL(frameIndexChanged(int)) , this , SLOT(frameIndexChanged(int)));
     //QObject::connect(_consumer , SIGNAL(setStartFrameIndex(int)) , _producer , SLOT(setStartFrameIndex(int)));
     //QObject::connect(_consumer , SIGNAL(requireFrames()) , _producer , SLOT(produceFrames()) );
     //QObject::connect(_consumer , SIGNAL(require1Frame()) , _producer , SLOT(produce1Frame()) );
 
     //QObject::connect(ui->openBtn , SIGNAL(pressed()) , this , SLOT(openBtnPressed()));
     //QObject::connect(ui->playBtn , SIGNAL(pressed()) , this , SLOT(playBtnPressed()));
     //QObject::connect(ui->stopBtn , SIGNAL(pressed()) , this , SLOT(stopBtnPressed()));
     //QObject::connect(ui->frameNoSpinBox , SIGNAL(valueChanged(int)) , this , SLOT(spinBoxValueChanged(int)));
     //QObject::connect(ui->frameNoSpinBox , SIGNAL(valueChanged(int)) , ui->playSlider , SLOT(setValue(int)));
     //QObject::connect(ui->playSlider , SIGNAL(valueChanged(int)) , this , SLOT(sliderValueChanged(int)));
     //QObject::connect(ui->playSlider , SIGNAL(valueChanged(int)) , ui->frameNoSpinBox , SLOT(setValue(int)));
 
     //QObject::connect(this , SIGNAL(startPlay()) , _consumer , SLOT(startPlay()) );
     //QObject::connect(this , SIGNAL(play1Frame(int)) , _consumer , SLOT(display1Frame(int)) );
     //QObject::connect(this , SIGNAL(stopPlay()) , _consumer , SLOT(stopPlay()) );
     //QObject::connect(this , SIGNAL(pausePlay()) , _consumer , SLOT(pausePlay()) );
 
}

void DDAPostPlayer::playBtnPressed()
{
    if(ui->playBtn->text()==QObject::tr("Play")){
        ui->playBtn->setText(QObject::tr("Pause"));
//         emit startPlay();
        _producer->produce1Frame();
    }
    else{
        ui->playBtn->setText(QObject::tr("Play"));
        emit pausePlay();
    }
}

void DDAPostPlayer::openBtnPressed()
{
    msg("open button pressed.\n");

    char subpath[] = "Mod\\DDA";
    char backpath[200];
    char path[200];
    _getcwd(path, 200);
    strcpy(backpath , path);
    msg(path);

    int len = strlen(path);
    int t=len-3;    //   ${FreeCAD}\\bin
//     msg(path+t);
    if ( path[t]=='b' && path[t+1]=='i' && path[t+2]=='n')
    {
        for( int i=0 ; i<8 ; i++)
            path[t++]=subpath[i];
        path[t]='\0';
//         msg(path);
//         msg("\n========\n");
    }
//     msg(path);
//     msg("\n========\n");

    if(!(_chdir(path)== 0)) return;

    _getcwd(path , 200);
    msg("\nchange to path:  ");
    msg(path);
    msg("\n========\n");

    FILE* fp = fopen("dg_ff.c" ,"rb");
    if(!fp) {msg("input dg file open failed.\n");  return ;}
    char content[200];
    fgets(content , 200 , fp);
    fclose(fp);

//    if(!_producer->setFile("D:/DDAProject/data.dg"))return;
     if(!_producer->setFile(content))return;
    FrameDataTaker* dataTaker = _producer->getFrameDataTaker();
    FrameMaker* maker = _consumer->getFrameMaker();
    maker->reset(dataTaker);
    initPlay(_producer->getTotalFramesNum());
}

void DDAPostPlayer::stopBtnPressed()
{
    emit stopPlay();
}

void DDAPostPlayer::frameIndexChanged(int frameIndex)
{
    ui->frameNoSpinBox->setValue(frameIndex);
    ui->playSlider->setValue(frameIndex);
    QSize qsize = _rootWidget->size();
    _soQtViewer->render();

    if(ui->playBtn->text()==QObject::tr("Pause"))
        _producer->produce1Frame();


    QTimer t;
    QEventLoop loop;
    QObject::connect(&t , SIGNAL(timeout()) , &loop , SLOT(quit()));
    t.setSingleShot(true);
    t.start(200);
    loop.exec();
}

void DDAPostPlayer::spinBoxValueChanged(int frameIndex)
{
    if(ui->frameNoSpinBox->hasFocus())
    {
        ui->playBtn->setText(QObject::tr("Play"));
        ui->frameNoSpinBox->clearFocus();
        msg("display specific frame from spinBox");
        emit play1Frame(frameIndex);
    }
}

void DDAPostPlayer::sliderValueChanged(int frameIndex)
{
    if(ui->playSlider->hasFocus())
    {
        ui->playBtn->setText(QObject::tr("Play"));
        ui->playSlider->clearFocus();
        msg("display specific frame from slider\n");
        emit play1Frame(frameIndex);
    }
}

bool DDAPostPlayer::eventFilter(QObject* target , QEvent* event)
{
    if(target == ui->playSlider)
    {
        if(event->type()==QEvent::MouseButtonRelease)
        {
            QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
            if(mouseEvent->buttons() & Qt::LeftButton)
            {
                int duration = ui->playSlider->maximum() - ui->playSlider->minimum();
                double scale = ((double)mouseEvent->x() / ui->playSlider->pos().x())/(double(duration));
                int pos = ui->playSlider->minimum() + int(duration*scale) ;
                ui->playSlider->setValue(pos);     
            }
            return true;
        }
    }
    return QWidget::eventFilter(target , event);
}
