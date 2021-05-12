#ifndef COMMUNICATON_H
#define COMMUNICATON_H

#include <QObject>

class communicaton : public QObject
{
    Q_OBJECT
public:
    explicit communicaton( QObject* parent = nullptr );

signals:
};

#endif // COMMUNICATON_H
