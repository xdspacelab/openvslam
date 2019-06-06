/*キャンバスのマウスの操作を整理するクラス
大きく2つの機能がある。
・マウスのボタンクリックとポインタ移動の検知
・マウスホイールの回転の検知
左右クリックやマウス移動など，処理に向かない形式で発行されるイベントを
扱いやすい形でリスナに返すクラス。
マウスホイールも扱うことができる。
*/

/*クリックイベント取得クラス*/
//コンストラクタにて与えられたIDを持つ要素に対し，マウスのイベントを監視する。
//リスナにはボタンとアクションが引数として渡される。
class MouseHandler {

    //コンストラクタ
    //キャンバスIDとリスナ関数を渡す
    //リスナの返り値にfalseを返すと，再びボタン押下までアクションは起きない。
    //リスナ関数に指定する引数：(btn,act,pos)
    //  btn : 押されたボタン，左クリック：0，右クリック：2
    //  act : マウスのアクション，プッシュ：0，移動：1，リリース：2，範囲外に移動：3
    //  pos : マウスアクションが発生した座標，型は二次元リスト
    //  return : このあとのイベントの発行を阻害したければfalseを返す
    constructor(domElement, listener) {
        let element = (domElement !== undefined) ? domElement : document;
        //各種マウスのアクションを監視し，mouse関数に投げる
        element.addEventListener('mousedown', function (e) {
            mouseHandler.mouse(e, 0);
        }, false);
        element.addEventListener('mousemove', function (e) {
            mouseHandler.mouse(e, 1);
        }, false);
        element.addEventListener('mouseup', function (e) {
            mouseHandler.mouse(e, 2);
        }, false);
        element.addEventListener('mouseout', function (e) {
            mouseHandler.mouse(e, 3);
        }, false);

        element.addEventListener('contextmenu', function (e) {
            e.preventDefault();
        }, false);

        this.listener = listener;

        //ボタンは押下時のみ判別可能なので，値を保持しておく必要あり
        this.btn = 0;
        //ボタン押下前のマウスホバーまたは，ボタン押下後マウスが範囲外に出て再び入ったあとのボタン開放
        //を検知しないようにするフラグ
        this.pushing = false;
        this.prevPos = [0, 0];
    }


    //何らかのマウスアクションを検出
    mouse(e, act) {

        //console.log(e.button + "," + act);

        //ターゲット，即ちキャンバスの左上の座標を取得し，マウスの座標から減ずることで
        //キャンバスの左上を基準とするポインタの座標posを得る。
        let rect = e.target.getBoundingClientRect();
        let pos = [e.clientX - rect.left, e.clientY - rect.top];

        //ボタン押下検知。ボタンの種類を保管し，リスナを起動。
        //リスナの返り値がfalseでなければenableをtrueに。
        if (act == 0) {
            if (this.pushing == false) {
                //すでにあるボタンが押されていた場合，以下のブロックは実行されない。
                this.btn = e.button;
                let continuous = this.listener(this.btn, act, pos, [0, 0]);
                this.pushing = (continuous == false) ? false : true;
                this.prevPos = pos;
            }
        }
        //マウスホバー検知。
        //ボタン押下後なら，マウスドラッグとして検知する。
        else if (act == 1) {
            if (this.pushing == true) {
                let dp = [pos[0] - this.prevPos[0], pos[1] - this.prevPos[1]];
                let continuous = this.listener(this.btn, act, pos, dp);
                this.pushing = (continuous == false) ? false : true;
                this.prevPos = pos;
            }
        }
        //ボタンリリース検知。
        //リスナを起動し，強制的にenableをクリアする。
        else if (act == 2) {
            //押されたときのボタンと異なるボタンが離されたときには以下のブロックを実行しない。
            if (this.pushing == true && this.btn == e.button) {
                let dp = [pos[0] - this.prevPos[0], pos[1] - this.prevPos[1]];
                this.listener(this.btn, act, pos, dp);
                this.pushing = false;
                this.prevPos = pos;
            }
        }
        //マウスアウトを検知。
        //マウスアウトした場合は強制的にボタンアクションを終了する。
        else {
            if (this.pushing == true) {
                let dp = [pos[0] - this.prevPos[0], pos[1] - this.prevPos[1]];
                this.listener(this.btn, act, pos, dp);
                this.pushing = false;
                this.prevPos = pos;
            }
        }
    }

}

/*マウスホイール回転の検知クラス*/
//コンストラクタで検知対象のIDとイベントリスナを指定する。
class WheelHandler {

    //コンストラクタ
    //検知対象のIDとイベントリスナを登録する
    constructor(domElement, listener) {

        let element = (domElement !== undefined) ? domElement : document;
        //ホイールイベント名を取得する。
        //ホイールイベント名は環境によって違う。
        let wheelEvent;
        if ('onwheel' in document) {
            wheelEvent = 'wheel';
        } else if ('onmousewheel' in document) {
            wheelEvent = 'mousewheel'
        } else {
            wheelEvent = 'DOMMouseScroll'
        }
        element.addEventListener(wheelEvent, function (e) {
            wheelHandler.onWheel(e)
        }, false);
        this.listener = listener;
    }
    //ホイール回転を検知するイベント
    onWheel(event) {
        event.preventDefault();

        let rot;

        switch (event.deltaMode) {
            case 2:
                rot = event.deltaY * 0.025;
                break;
            case 1:
                // Zoom in lines
                rot = event.deltaY * 0.01;
                break;
            default:
                // undefined, 0, assume pixels
                rot = event.deltaY * 0.00025;
                break;
        }

        this.listener(rot);
    }
}
