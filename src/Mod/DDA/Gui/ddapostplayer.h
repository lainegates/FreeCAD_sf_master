#ifndef DDAPOSTPLAYER_H
#define DDAPOSTPLAYER_H

#include <QWidget>
#include <qtcore/qthread.h>
#include <qtcore/QObject>
#include <qtcore/QMutex>
#include <QtCore/QSize>
// #include <QtCore/qtimer.h>
#include "GraphTools.h"
#include "FrameMaker.h"
// #include <Gui/View3DInventor.h>
// #include <Gui/View3DInventorViewer.h>


#include <Inventor/Qt/viewers/SoQtViewer.h>

class View3DInventorViewer;

namespace Ui {
class DDAPostPlayer;
}

namespace DDAGui{
// 
// #ifndef  PRODUCER_CONSUMER
// #define  PRODUCER_CONSUMER
//using namespace DDAGui;
//#define DEBUGPRODUCER
class FrameProducer: public QObject
{
    Q_OBJECT
public:
    FrameProducer(){ _dataTaker = new FrameDataTaker;}

Q_SIGNALS:
    void noMoreFrames();
    void oneFrameProduced();

public Q_SLOTS:
    void produce1Frame();
    void produceFrames();
    bool setFile(char* filename){return _dataTaker->setFile(filename);}
    void setStartFrameIndex(int frameIndex);

public:
    int getTotalFramesNum(){ return _dataTaker->getSchema()[6];}
    FrameDataTaker* getFrameDataTaker() {return _dataTaker;}

private:
    void _realProduce1Frame();

private:
    FrameDataTaker* _dataTaker;
    QMutex _lock;
    QMutex _assistLock;
};

//#define DEBUGCONSUMER
class FrameConsumer: public QObject
{
    Q_OBJECT
public:
    FrameConsumer():_ifContinuePlay(true){_frameMaker = new FrameMaker;}

Q_SIGNALS:
    void requireFrames();
    void require1Frame();
    void frameIndexChanged(int frameIndex);
    void setStartFrameIndex(int frameIndex);

public Q_SLOTS:
    void displayFrames();
    void display1Frame(int frameIndex);
    void stopPlay();
    void startPlay();
    void pausePlay();
    void acceptPlayEnd();

public:
    void setRootSceneNode(SoSeparator* root){_frameMaker->setRootSceneNode(root); _root = root;}
    void setRootQWidget(QWidget* widget){
//         QObject::connect(this , SIGNAL(windowRepaint()) , widget , SLOT(update()));
//         QObject::connect(this , SIGNAL(windowRepaint2(int , int , int , int))
//             , widget , SLOT(repaint(int, int, int, int)) , Qt::QueuedConnection);
    }

    FrameMaker* getFrameMaker(){return _frameMaker;}

private: 

private:
    bool _ifContinuePlay;
    bool _ifProduceDone;
    QMutex _lock;
    QMutex _assistLock;
    FrameMaker* _frameMaker;
    SoSeparator* _root;
};
// #endif // PRODUCER_CONSUMER
// 
 } //DDAGui

//#define DEBUGPOSTPLAYER
class DDAPostPlayer : public QWidget
{
    Q_OBJECT
public:
    explicit DDAPostPlayer(QWidget *parent = 0);
    ~DDAPostPlayer();
    
public Q_SLOTS:
    void frameIndexChanged(int frameIndex);

    void playBtnPressed();
    void openBtnPressed();
    void stopBtnPressed();

private Q_SLOTS:
    void spinBoxValueChanged(int value);
    void sliderValueChanged(int value);

Q_SIGNALS:
    void startPlay();
    void play1Frame(int frameIndex);
    void stopPlay();
    void pausePlay();

public:
    void setRootSceneNode(SoSeparator* root){_consumer->setRootSceneNode(root);}
    void setRootQWidget(QWidget* widget){_consumer->setRootQWidget(widget);_rootWidget = widget;}
    void setSoQtViewer(SoQtViewer* viewer){_soQtViewer = viewer;}

private:
    void initPlay(int framesNum);
    void initConnection();

protected:
    bool eventFilter(QObject* target , QEvent* event);

private:
    DDAGui::FrameProducer* _producer;
    DDAGui::FrameConsumer* _consumer;
//     SoSeparator* _rootScene;
    QWidget* _rootWidget;
    SoQtViewer* _soQtViewer;

    QThread* _produceThread;
    QThread* _consumeThread;
    Ui::DDAPostPlayer *ui;
};

#endif // DDAPOSTPLAYER_H
