#ifndef TINYRENDERER_H
#define TINYRENDERER_H

#include <QMainWindow>

namespace Ui {
class tinyRenderer;
}

class tinyRenderer : public QMainWindow
{
    Q_OBJECT

public:
    explicit tinyRenderer(QWidget *parent = 0);
    ~tinyRenderer();

private:
    Ui::tinyRenderer *ui;
};

#endif // TINYRENDERER_H
