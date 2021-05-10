#include "sqlhandler.h"
#include <iostream>
#include <fstream>
#include <string>
#include <QMessageBox>

sqlHandler::sqlHandler()
{
    config();
    checkSimNumber();
    setupTable();
}

void sqlHandler::config()
{
    std::ifstream dataBasePassFile;
    dataBasePassFile.open("ssap.txt");
    if(!dataBasePassFile.is_open())
    {
        QMessageBox info;
        info.setText("No pass file");
        info.setWindowTitle("Pass file error");
        info.exec();
        throw "noPassFile";
    }
    std::string p;
    dataBasePassFile >> p;
    QString pass(p.c_str());
    dataBasePassFile.close();
    dataBase = QSqlDatabase::addDatabase("QMYSQL");
    dataBase.setHostName("mysql.agh.edu.pl");
    dataBase.setPort(3306);
    dataBase.setDatabaseName("aghmari1");
    dataBase.setUserName("aghmari1");
    dataBase.setPassword(pass);
    if( !dataBase.open())
    {
        QMessageBox info;
        info.setText("Can't connect to database");
        info.setWindowTitle("Connection error");
        info.exec();
        throw "noConnection";
    }
    workingTable = new QSqlTableModel(nullptr,dataBase);
}

void sqlHandler::checkSimNumber()
{
    auto presentSimNames = dataBase.tables();

    for(bool isAvailableName = false;isAvailableName==false;)
    {
        isAvailableName = true;
        for(const auto &in:presentSimNames)
        {
            //std::cout<<in.toStdString();
            if(simUploadName + static_cast<QString>(simNumber) == in)
            {
                isAvailableName = false;
            }
            ++simNumber;
        }
    }
    simUploadName.append(QString::number(simNumber));
}

void sqlHandler::setupTable()
{
    query = QSqlQuery(dataBase);
    dataBase.transaction();
    QString tableInit = QString("CREATE TABLE Simulation%1 (Time DOUBLE primary key, PosX DOUBLE, PosY DOUBLE, PosZ DOUBLE, RotX DOUBLE, RotY DOUBLE, RotZ DOUBLE);").arg(simNumber);
    query.exec(tableInit);
    dataBase.commit();
    //
    //dataBase.
    /*workingTable->setTable(simUploadName);
    workingTable->setEditStrategy(QSqlTableModel::OnManualSubmit);
    workingTable->insertRows(0,10000);
    //dataBase.commit();*/
    //workingTable->select();

    /*workingTable->setHeaderData(0,Qt::Horizontal,tr("PositionX"));
    workingTable->setHeaderData(1,Qt::Horizontal,tr("PositionY"));
    workingTable->setHeaderData(2,Qt::Horizontal,tr("PositionZ"));
    workingTable->setHeaderData(3,Qt::Horizontal,tr("RotationX"));
    workingTable->setHeaderData(4,Qt::Horizontal,tr("RotationY"));
    workingTable->setHeaderData(5,Qt::Horizontal,tr("RotationZ"));*/
    //workingTable.setHeaderData(1,Qt::Horizontal,tr("Camera View"));
   /* workingTable->insertRows(0,1);
    workingTable->setData(workingTable->index(0,0),"0.0");
    workingTable->setData(workingTable->index(0,1),"1.0");
    workingTable->setData(workingTable->index(0,2),"2.0");
    workingTable->setData(workingTable->index(0,3),"3.0");
    workingTable->setData(workingTable->index(0,4),"4.0");
    workingTable->setData(workingTable->index(0,5),"5.0");
    workingTable->database().transaction();
    workingTable->submitAll();
    workingTable->database().commit();*/
}
void sqlHandler::addLineOfData(double &timeElapsed, VectorXd &position, VectorXd &thrusterAzimuth)
{
    tableUpd.append(QString("(%1, %2, %3, %4, %5, %6, %7), ").arg(timeElapsed).arg(position[0]).arg(position[1]).arg(position[2]).arg(position[3]).arg(position[4]).arg(position[5]));

}
void sqlHandler::sendSimDataToServer(double timeElapsed, VectorXd position, VectorXd thrusterAzimuth)
{
    if(tableRows==0)
    {
        tableUpd = QString("INSERT INTO Simulation%1 (Time, PosX, PosY, PosZ, RotX, RotY, RotZ) VALUES ").arg(simNumber);
        addLineOfData(timeElapsed, position, thrusterAzimuth);
        ++tableRows;
    }
    else if(tableRows>49)
    {
        tableUpd.chop(2);
        dataBase.transaction();
        query.exec(tableUpd);
        //std::cout<<tableUpd.toStdString()<<"\n";
        dataBase.commit();
        tableRows = 0;
    }
    else
    {
        addLineOfData(timeElapsed, position, thrusterAzimuth);
        ++tableRows;
    }
    //workingTable->select();
    /*workingTable->setData(workingTable->index(tableRow,0),timeElapsed);
    workingTable->setData(workingTable->index(tableRow,1),position[0]);
    workingTable->setData(workingTable->index(tableRow,2),position[1]);
    workingTable->setData(workingTable->index(tableRow,3),position[2]);
    workingTable->setData(workingTable->index(tableRow,4),position[3]);
    workingTable->setData(workingTable->index(tableRow,5),position[4]);
    workingTable->setData(workingTable->index(tableRow,6),position[5]);*/

    //dataBase.transaction();


    //tableUpd.append(
    //query.exec();
    //dataBase.commit();

//if(static_cast<int>(timeElapsed*1000)%5000==0)
//{
    //query.exec(tableUpd);
    //workingTable->database().transaction();
   // workingTable->submitAll();

    //workingTable->database().commit();
    //dataBase.transaction();
    //dataBase.commit();
//}



    //std::cout<<timeElapsed<<"\n";
    //++tableRow;
}

sqlHandler::~sqlHandler()
{
    dataBase.close();
}
