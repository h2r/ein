#include "stackmodel.h"


typedef enum {
  IDX = 0,
  NAME = 1,
  DESCRIPTION = 2,
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
    } else if (index.column() == DESCRIPTION) {
      return QString::fromStdString(w->description());
    } else {
      assert(0);
    }
  }
}


QVariant StackModel::headerData(int section /* section */,
                                Qt::Orientation orientation /* orientation */,
                                int role) const
{
  if (orientation == Qt::Vertical) {
    return QVariant();
  }

  if (role == Qt::SizeHintRole) {
    if (section == IDX) {
      return QSize(10, 20);
    } else if (section == NAME) {
      return QSize(50, 20);
    } else if (section == DESCRIPTION) {
      return QSize(100, 1);
    } else {
      cout << "Unknown section: " << section << endl;
      assert(0);
    }
  } else if (role == Qt::DisplayRole) {
    if (section == IDX) {
      return "Idx";
    } else if (section == NAME) {
      return "Name";
    } else if (section == DESCRIPTION) {
      return "Description";
    } else {
      cout << "Unknown section: " << section << endl;
      assert(0);
    }
  } else {
    return QVariant();
  }
}
