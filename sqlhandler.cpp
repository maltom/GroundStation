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
    dataBasePassFile.open( "ssap.txt" );
    if( !dataBasePassFile.is_open() )
    {
        QMessageBox info;
        info.setText( "No pass file" );
        info.setWindowTitle( "Pass file error" );
        info.exec();
        throw "noPassFile";
    }
    std::string p;
    dataBasePassFile >> p;
    QString pass( p.c_str() );
    dataBasePassFile.close();
    dataBase = QSqlDatabase::addDatabase( "QMYSQL" );
    dataBase.setHostName( "mysql.agh.edu.pl" );
    dataBase.setPort( 3306 );
    dataBase.setDatabaseName( "aghmari1" );
    dataBase.setUserName( "aghmari1" );
    dataBase.setPassword( pass );
    if( !dataBase.open() )
    {
        QMessageBox info;
        info.setText( "Can't connect to database" );
        info.setWindowTitle( "Connection error" );
        info.exec();
        throw "noConnection";
    }
    workingTable = new QSqlTableModel( nullptr, dataBase );
}

void sqlHandler::checkSimNumber()
{
    auto presentSimNames = dataBase.tables();

    for( bool isAvailableName = false; isAvailableName == false; )
    {
        isAvailableName = true;
        for( const auto& in : presentSimNames )
        {
            // std::cout<<in.toStdString();
            if( simUploadName + static_cast< QString >( simNumber ) == in )
            {
                isAvailableName = false;
            }
            ++simNumber;
        }
    }
    simUploadName.append( QString::number( simNumber ) );
}

void sqlHandler::setupTable()
{
    query = QSqlQuery( dataBase );
    dataBase.transaction();
    QString tableInit = QString( "CREATE TABLE Simulation%1 (Time DOUBLE primary key, PosX DOUBLE, PosY DOUBLE, PosZ "
                                 "DOUBLE, RotX DOUBLE, RotY DOUBLE, RotZ DOUBLE);" )
                            .arg( simNumber );
    query.exec( tableInit );
    dataBase.commit();
}
void sqlHandler::addLineOfData( double& timeElapsed, VectorXd& position, VectorXd& thrusterAzimuth )
{
    tableUpd.append( QString( "(%1, %2, %3, %4, %5, %6, %7), " )
                         .arg( timeElapsed )
                         .arg( position[ 0 ] )
                         .arg( position[ 1 ] )
                         .arg( position[ 2 ] )
                         .arg( position[ 3 ] )
                         .arg( position[ 4 ] )
                         .arg( position[ 5 ] ) );
}
void sqlHandler::sendSimDataToServer( double timeElapsed, VectorXd position, VectorXd thrusterAzimuth )
{
    if( tableRows == 0 )
    {
        tableUpd
            = QString( "INSERT INTO Simulation%1 (Time, PosX, PosY, PosZ, RotX, RotY, RotZ) VALUES " ).arg( simNumber );
        addLineOfData( timeElapsed, position, thrusterAzimuth );
        ++tableRows;
    }
    else if( tableRows > 49 )
    {
        tableUpd.chop( 2 );
        dataBase.transaction();
        query.exec( tableUpd );
        // std::cout<<tableUpd.toStdString()<<"\n";
        dataBase.commit();
        tableRows = 0;
    }
    else
    {
        addLineOfData( timeElapsed, position, thrusterAzimuth );
        ++tableRows;
    }
}

sqlHandler::~sqlHandler()
{
    dataBase.close();
}
