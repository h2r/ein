#ifndef STACKMODEL_H
#define STACKMODEL_H


#include <QtWidgets>
#include <QAbstractTableModel>

#include "ein.h"

class StackModel : public QAbstractTableModel
{
    Q_OBJECT

 public:
    StackModel(QObject *parent = 0);

    void setMachineState(shared_ptr<MachineState> _ms);

    int rowCount(const QModelIndex &parent = QModelIndex()) const Q_DECL_OVERRIDE;
    int columnCount(const QModelIndex &parent = QModelIndex()) const Q_DECL_OVERRIDE;

    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const Q_DECL_OVERRIDE;
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const Q_DECL_OVERRIDE;

 private:
    shared_ptr<MachineState> ms;
};


#endif // STACKMODEL_H
