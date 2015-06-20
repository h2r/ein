#include "stackmodel.h"


typedef enum {
  IDX = 0,
  NAME = 1,
  REPR = 2,
} columns;

StackModel::StackModel(QObject *parent)
  : QAbstractTableModel(parent)
{
}

void StackModel::setMachineState(shared_ptr<MachineState> _ms)
{
  beginResetModel();
  ms = _ms;
  endResetModel();
}




int StackModel::rowCount(const QModelIndex & /* parent */) const
{
  return ms->call_stack.size();
}

int StackModel::columnCount(const QModelIndex & /* parent */) const
{
  return 3;
}


QVariant StackModel::data(const QModelIndex &index, int role) const
{
  if (!index.isValid() || role != Qt::DisplayRole) {
    return QVariant();
  } else {
    shared_ptr<Word> w = ms->call_stack[ms->call_stack.size() - index.row() - 1];
    if (index.column() == IDX) {
      return index.row();
    } else if (index.column() == NAME) {
      return QString::fromStdString(w->name());
    } else if (index.column() == REPR) {
      return QString::fromStdString(w->repr());
    } else {
      assert(0);
    }
  }
}


QVariant StackModel::headerData(int section /* section */,
                                Qt::Orientation orientation /* orientation */,
                                int role) const
{
  if (role == Qt::SizeHintRole) {
    return QSize(1, 3);
  }

  if (orientation == Qt::Vertical) {
    return QVariant();
  } else  if (orientation == Qt::Horizontal) {
    if (section == IDX) {
      return "Idx";
    } else if (section == NAME) {
      return "Name";
    } else if (section == REPR) {
      return "Repr";
    } else {
      cout << "Unknown section: " << section << endl;
      assert(0);
    }
  } else {
    cout << "Unknown orientation: " << orientation << endl;
    assert(0);
  }


}
