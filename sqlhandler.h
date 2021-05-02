#ifndef SQLHANDLER_H
#define SQLHANDLER_H

#include <Eigen/Dense>

#include <QObject>
#include <QtSql>
#include <QSqlDatabase>
#include <QSqlDriver>

using namespace Eigen;
class sqlHandler : public QObject
{
    Q_OBJECT
public:
    sqlHandler();
    ~sqlHandler();

private:
    void config();
    void setupTable();
    void checkSimNumber();
    void setColumns();
    void enterData();
    void sendData();
    QSqlDatabase dataBase;
    QString simUploadName{"Simulation"};
    unsigned simNumber {0};
    unsigned tableRows {0};
    QSqlTableModel* workingTable;
    QSqlQuery query;
    QString tableUpd;
public slots:
    void addLineOfData(double &timeElapsed, VectorXd &position, VectorXd &thrusterAzimuth);
    void sendSimDataToServer(double timeElapsed, VectorXd position, VectorXd thrusterAzimuth);

};

#endif // SQLHANDLER_H
