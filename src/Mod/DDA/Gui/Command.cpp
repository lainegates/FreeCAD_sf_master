/***************************************************************************
 *   Copyright (c) YEAR YOUR NAME         <Your e-mail address>            *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Library General Public           *
 *   License as published by the Free Software Foundation; either          *
 *   version 2 of the License, or (at your option) any later version.      *
 *                                                                         *
 *   This library  is distributed in the hope that it will be useful,      *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Library General Public License for more details.                  *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this library; see the file COPYING.LIB. If not,    *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA  02111-1307, USA                                *
 *                                                                         *
 ***************************************************************************/


#include "PreCompiled.h"
#ifndef _PreComp_
#endif

#include <ctime>
//#include <Gui/SoFCUnifiedSelection.h>

#include <qtcore/QObject>
#include <QtCore/QThread>
#include <qtcore/QString>
#include <QtCore/QTimer>
#include <qtcore/QEventLoop>

#include <QtGui/QPushButton>

#include "FrameReader.h"
#include "GraphTools.h"
#include "FrameMaker.h"
#include "ddapostplayer.h"
#include <stdio.h>
#include <errno.h>
#include <winbase.h>

#include <Inventor/Qt/viewers/SoQtViewer.h>

#include <Base/Console.h>
#include <App/Document.h>
#include <Gui/Application.h>
#include <Gui/Command.h>
#include <Gui/Document.h>
#include <Gui/View3DInventor.h>
#include <Gui/View3DInventorViewer.h>
#include <Gui/Control.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

namespace DDAGui{
	bool DEBUG = true;

    void sleep( double seconds )
    {
        QTimer t;
        QEventLoop eventloop;
        QObject::connect(&t , SIGNAL(timeout()) , &eventloop , SLOT(quit()));
        t.setSingleShot(true);
        t.start(seconds*10); // make a pause of 20 s
        eventloop.exec(); // stops here until the timeout() signal is emitted
    }

	void msg2File(const char* str)
	{
		FILE* fp= fopen("D:/freecad_log.txt", "ab");
		fprintf( fp ,str);
		fclose(fp);
	}

	void msg(const char* str)
	{
		if (DEBUG)
		{
			Base::Console().Message(str);
			//msg2File(str);
		}
	}

    void msgString(const string& str)
    {
        FILE* fp= fopen("D:/freecad_log.txt", "ab");
        fwrite(str.c_str() , sizeof(char) , str.size() , fp);
        fclose(fp);
    }

	void msgQString(QString str)
	{
		if(DEBUG)
			msg(str.toStdString().c_str());
	}
	void saveTime()
	{
		char currentTime[200];
		time_t rawtime;
		struct tm * timeinfo;

		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		sprintf ( currentTime , "\n\n**************************************\n*********   %s**************************************\n", asctime (timeinfo) );
		msg2File(currentTime);
	}

}
using namespace DDAGui;


int sum = 0;

void testFrameReader()
{
	FrameReader reader;
	reader.setFile("D:/DDAProject/data.dg");
 	std::vector<int> schema = reader.getSchema();
	std::vector<double> windowInfo = reader.getWindowInfo();
 	char str[200];
 	sprintf(str , "%d %d %d %d %d %d %d\n%lf %lf %lf %lf\n" , int(schema[0]) , int(schema[1]) , int(schema[2])
 		, int(schema[3]) , int(schema[4]) , int(schema[5]) , int(schema[6]) 
 		, windowInfo[0] , windowInfo[1] , windowInfo[2] , windowInfo[3]);
 	msg(str);
	string tmpStr = reader.getNextFrame();
	while(tmpStr.size()>0){ /*msgQString(tmpStr);*/ tmpStr = reader.getNextFrame();}
}

void testFrameDataTaker()
{
	FrameDataTaker dataTaker;
    bool flag = dataTaker.setFile("D:/DDAProject/data.dg");
	assert(flag);
	FrameData graph = dataTaker.getNextGraph();
}


SoSeparator* getFreeCADRootNode()
{
    Gui::Document* doc = Gui::Application::Instance->activeDocument();
    assert(doc);
    Gui::View3DInventor* view = dynamic_cast<Gui::View3DInventor*>(doc->getActiveView());
    assert(view);
    if (view) {
        Gui::View3DInventorViewer* viewer = view->getViewer();
        SoNode* rootNode =  viewer->getSceneGraph();
        SoSeparator* node = static_cast<SoSeparator*>(rootNode);
        assert(node);
        msg("root scene graph node got.\n");
        return node;
    }
}

QWidget* getRootWidget()
{
    Gui::Document* doc = Gui::Application::Instance->activeDocument();
    assert(doc);
    Gui::View3DInventor* view = dynamic_cast<Gui::View3DInventor*>(doc->getActiveView());
    assert(view);
    QWidget* node = dynamic_cast<QWidget*>(view);
    assert(node);
    return node;
}

SoQtViewer* getSoQtViewer()
{
    Gui::Document* doc = Gui::Application::Instance->activeDocument();
    assert(doc);
    Gui::View3DInventor* view = dynamic_cast<Gui::View3DInventor*>(doc->getActiveView());
    assert(view);
    if (view) {
        Gui::View3DInventorViewer* viewer = view->getViewer();
        SoQtViewer* node = static_cast<SoQtViewer*>(viewer);
        assert(node);
        msg("SoqtViewer got.\n");
        return node;
    }
}

void testFrameMaker()
{
	FrameDataTaker dataTaker;
	assert(dataTaker.setFile("D:/DDAProject/data.dg"));

    msg("start to set frame maker.\n");
	FrameMaker frameMaker;
    SoSeparator* root = getFreeCADRootNode();
    assert(root);
	frameMaker.setRootSceneNode(root);
    msg("start to reset frame maker.\n");
	frameMaker.reset(&dataTaker);
    vector<int> schema = dataTaker.getSchema();
    for(int i=0; i<1000; i++)
    {
        sleep(0.5);
        msg("----------show one graph----------------.\n");
        frameMaker.updateGraph(dataTaker.getNextGraph());
    }
// 
}

void testPostPlayer()
{
    DDAPostPlayer* player = new DDAPostPlayer;

    SoSeparator* root = getFreeCADRootNode();
    assert(root);
    player->setRootSceneNode(root);

    QWidget* widget= getRootWidget();
    assert(widget);
    player->setRootQWidget(widget);

    SoQtViewer* viewer = getSoQtViewer();
    assert(viewer);
    player->setSoQtViewer(viewer);

    player->show();
}
DDAPostPlayer* postPlayer = NULL;
void startPostPlayer()
{
    if(!postPlayer) postPlayer= new DDAPostPlayer;
    SoSeparator* root = getFreeCADRootNode();
    assert(root);
    postPlayer->setRootSceneNode(root);

    QWidget* widget= getRootWidget();
    assert(widget);
    postPlayer->setRootQWidget(widget);

    SoQtViewer* viewer = getSoQtViewer();
    assert(viewer);
    postPlayer->setSoQtViewer(viewer);

    postPlayer->show();
}


//===========================================================================
// CmdDDATest THIS IS JUST A TEST COMMAND
//===========================================================================
DEF_STD_CMD(CmdDDATest);

CmdDDATest::CmdDDATest()
:Command("DDA_Test")
{
    sAppModule    = "DDA";
    sGroup        = QT_TR_NOOP("DDA");
    sMenuText     = QT_TR_NOOP("Hello");
    sToolTipText  = QT_TR_NOOP("DDA Test function");
    sWhatsThis    = QT_TR_NOOP("DDA Test function");
    sStatusTip    = QT_TR_NOOP("DDA Test function");
    sPixmap       = "Test1";
    sAccel        = "CTRL+H";
}



void CmdDDATest::activated(int iMsg)
{
//     assert(SoDB::isMultiThread());
// 	testFrameDataTaker();
// 	msg("one graph read done.\n");
//     testFread();

	saveTime();
    msg("start to read graph.\n");

//     testPostPlayer();
    startPostPlayer();

    Base::Console().Message("Hello, World!\n");
}

void CreateDDACommands(void)
{
    Gui::CommandManager &rcCmdMgr = Gui::Application::Instance->commandManager();
    rcCmdMgr.addCommand(new CmdDDATest());
}
