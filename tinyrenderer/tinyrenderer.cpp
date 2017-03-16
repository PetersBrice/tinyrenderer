#include "tinyrenderer.h"
#include "ui_tinyrenderer.h"

tinyRenderer::tinyRenderer(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::tinyRenderer)
{
    ui->setupUi(this);
}

tinyRenderer::~tinyRenderer()
{
    delete ui;
}
