#ifndef EDITORVIEW_H
#define EDITORVIEW_H

#include <QObject>
#include <QQmlApplicationEngine>

#include "DataTypes.h"
#include "IModelService.h"
#include "IObserver.h"
#include "ViewController.h"

class ViewController;

class EditorView : public QObject, public IObserver {
    Q_OBJECT

public:
    explicit EditorView(IModelService * pModelService, ViewController * pViewController, QQmlApplicationEngine * pAppEngine, QObject * pParent = 0);
    ~EditorView();

    void notify();

private:
	ViewController          * m_pViewController;
    IModelService           * m_pModelService;
    QQmlApplicationEngine   * m_pAppEngine;

};

#endif // EDITORVIEW_H