using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class CollisionManager : MonoBehaviour
{
    public GameObject object1; // 第一个物体
    public GameObject object2; // 第二个物体

    public GameObject trigger_object; // trigger物体

    public TextMeshProUGUI collisionCountText; // 显示碰撞次数的UI文本

    private int collisionCount = 0; // 碰撞次数

    private Renderer object1Renderer;
    private Renderer object2Renderer;
    private Material object1OriginalMaterial;
    private Material object2OriginalMaterial;

    private void Start()
    {
        // 获取物体的Renderer组件
        object1Renderer = object1.GetComponent<Renderer>();
        object2Renderer = object2.GetComponent<Renderer>();

        // 保存物体的原始材质
        object1OriginalMaterial = object1Renderer.material;
        object2OriginalMaterial = object2Renderer.material;

        collisionCountText.text = "Collision Count: " + collisionCount.ToString();
    }

    private void OnTriggerEnter(Collider other)
    {
        // 检测碰撞的两个物体是否是指定的物体
        if (other.gameObject == trigger_object)
        {
            // 高亮两个物体
            HighlightObject(object1);
            HighlightObject(object2);

            // 增加碰撞次数
            collisionCount++;
            collisionCountText.text = "Collision Count: " + collisionCount.ToString();
        }

    }

    private void OnTriggerExit(Collider other)
    {
        // 检测碰撞的两个物体是否是指定的物体
        if (other.gameObject == trigger_object)
        {
            // 取消高亮两个物体，恢复原样
            ResetHighlight(object1);
            ResetHighlight(object2);
        }
    }

    private void HighlightObject(GameObject obj)
    {
        // 创建一个新的材质，并设置高亮颜色
        Material highlightMaterial = new Material(object1OriginalMaterial);
        highlightMaterial.color = Color.yellow;

        // 将新的材质应用到物体上
        if (obj == object1)
        {
            object1Renderer.material = highlightMaterial;
        }
        else if (obj == object2)
        {
            object2Renderer.material = highlightMaterial;
        }
    }

    private void ResetHighlight(GameObject obj)
    {
        // 恢复原始材质
        if (obj == object1)
        {
            object1Renderer.material = object1OriginalMaterial;
        }
        else if (obj == object2)
        {
            object2Renderer.material = object2OriginalMaterial;
        }
    }



    public void ResetCollisionCount()
    {
        // 重置碰撞次数，并更新UI文本
        collisionCount = 0;
        collisionCountText.text = "Collision Count: " + collisionCount.ToString();
    }
}