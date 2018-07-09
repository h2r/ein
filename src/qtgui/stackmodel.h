#ifndef STACKMODEL_H
#define STACKMODEL_H


#include <QtWidgets>
#include <QAbstractTableModel>


using namespace std;

#include <vector>
#include <memory>

class Word;

class StackModel : public QAbstractTableModel
{
    Q_OBJECT

 public:
    StackModel(QObject *parent = 0);

    void setStack(vector<shared_ptr<Word> > stack);

    int rowCount(const QModelIndex &parent = QModelIndex()) const Q_DECL_OVERRIDE;
    int columnCount(const QModelIndex &parent = QModelIndex()) const Q_DECL_OVERRIDE;

    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const Q_DECL_OVERRIDE;
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const Q_DECL_OVERRIDE;

 private:
    vector<shared_ptr<Word> > stack;
};


#endif // STACKMODEL_H
