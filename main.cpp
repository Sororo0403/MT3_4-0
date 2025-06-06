#define NOMINMAX
#include <Novice.h>
#include <cstdint>
#define _USE_MATH_DEFINES
#include <cmath>
#include <imgui.h>
#include <algorithm> // std::clamp, std::absを使うために必要
#include <vector>    // std::vector を使うために必要
#include <string>    // std::string を使うために必要

// ウィンドウの横幅
static const float kWindowWidth = 1280.0f;
// ウィンドウの縦幅
static const float kWindowHeight = 720.0f;

// タイトル
const char kWindowTitle[] = "LE2B_19_ハタナカ_タロウ - Spring Ball Simulation";

/// <summary>
/// 4x4行列
/// </summary>
struct Matrix4x4 {
	float m[4][4];
};

/// <summary>
/// 3次元ベクトル
/// </summary>
struct Vector3 {
	float x;
	float y;
	float z;
};

// Vector3 ヘルパー関数
/// <summary>
/// ベクトル同士の加算
/// </summary>
Vector3 Add(const Vector3& v1, const Vector3& v2) {
	return { v1.x + v2.x, v1.y + v2.y, v1.z + v2.z };
}

/// <summary>
/// ベクトル同士の減算
/// </summary>
Vector3 Subtract(const Vector3& v1, const Vector3& v2) {
	return { v1.x - v2.x, v1.y - v2.y, v1.z - v2.z };
}

/// <summary>
/// ベクトルのスカラー倍
/// </summary>
Vector3 MultiplyScalar(const Vector3& v, float scalar) {
	return { v.x * scalar, v.y * scalar, v.z * scalar };
}

/// <summary>
/// ベクトルの長さを計算
/// </summary>
float Length(const Vector3& v) {
	return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

/// <summary>
/// ベクトルを正規化
/// </summary>
Vector3 Normalize(const Vector3& v) {
	float len = Length(v);
	if (std::abs(len) > 1e-6f) { // ゼロ除算を避ける
		return { v.x / len, v.y / len, v.z / len };
	}
	return { 0.0f, 0.0f, 0.0f }; // 長さがゼロの場合はゼロベクトルを返す
}

/// <summary>
/// ばねの情報を保持する構造体
/// </summary>
struct Spring {
	Vector3 anchor;      // アンカーポイントの位置（ワールド座標）
	float naturalLength; // ばねの自然長
	float stiffness;     // ばね定数 (k)
	float dampingCoefficient; // 減衰係数 (c)
};

/// <summary>
/// ボールの情報を保持する構造体
/// </summary>
struct Ball {
	Vector3 position; // ボールの位置（ワールド座標）
	Vector3 velocity; // ボールの速度
	float mass;       // ボールの質量
	float radius;     // ボールの半径
	uint32_t color;   // ボールの色
};


/// <summary>
/// 単位行列を作成
/// </summary>
Matrix4x4 MakeIdentity4x4() {
	Matrix4x4 result;
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			result.m[i][j] = (i == j) ? 1.0f : 0.0f;
		}
	}
	return result;
}

/// <summary>
/// 4x4行列同士を乗算
/// </summary>
Matrix4x4 Multiply(const Matrix4x4& m1, const Matrix4x4& m2) {
	Matrix4x4 result;
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			result.m[i][j] = 0.0f;
			for (int k = 0; k < 4; ++k) {
				result.m[i][j] += m1.m[i][k] * m2.m[k][j];
			}
		}
	}
	return result;
}

/// <summary>
/// x軸回転行列を作成
/// </summary>
Matrix4x4 MakeRotateXMatrix(float radian) {
	Matrix4x4 matrix = MakeIdentity4x4();
	matrix.m[1][1] = std::cos(radian);
	matrix.m[1][2] = std::sin(radian);
	matrix.m[2][1] = -std::sin(radian);
	matrix.m[2][2] = std::cos(radian);
	return matrix;
}

/// <summary>
/// y軸回転行列を作成
/// </summary>
Matrix4x4 MakeRotateYMatrix(float radian) {
	Matrix4x4 matrix = MakeIdentity4x4();
	matrix.m[0][0] = std::cos(radian);
	matrix.m[0][2] = -std::sin(radian);
	matrix.m[2][0] = std::sin(radian);
	matrix.m[2][2] = std::cos(radian);
	return matrix;
}

/// <summary>
/// z軸回転行列を作成
/// </summary>
Matrix4x4 MakeRotateZMatrix(float radian) {
	Matrix4x4 matrix = MakeIdentity4x4();
	matrix.m[0][0] = std::cos(radian);
	matrix.m[0][1] = std::sin(radian);
	matrix.m[1][0] = -std::sin(radian);
	matrix.m[1][1] = std::cos(radian);
	return matrix;
}

/// <summary>
/// 平行移動行列の作成
/// </summary>
Matrix4x4 MakeTranslateMatrix(const Vector3& translate) {
	Matrix4x4 matrix = MakeIdentity4x4();
	matrix.m[3][0] = translate.x;
	matrix.m[3][1] = translate.y;
	matrix.m[3][2] = translate.z;
	return matrix;
}

/// <summary>
/// スケーリング行列の作成
/// </summary>
Matrix4x4 MakeScaleMatrix(const Vector3& scale) {
	Matrix4x4 matrix = MakeIdentity4x4();
	matrix.m[0][0] = scale.x;
	matrix.m[1][1] = scale.y;
	matrix.m[2][2] = scale.z;
	return matrix;
}

/// <summary>
/// アフィン変換行列を作成 (S * R * T)
/// </summary>
Matrix4x4 MakeAffineMatrix(const Vector3& scale, const Vector3& rotate, const Vector3& translate) {
	Matrix4x4 scaleMatrix = MakeScaleMatrix(scale);
	Matrix4x4 rotateXMatrix = MakeRotateXMatrix(rotate.x);
	Matrix4x4 rotateYMatrix = MakeRotateYMatrix(rotate.y);
	Matrix4x4 rotateZMatrix = MakeRotateZMatrix(rotate.z);
	Matrix4x4 translateMatrix = MakeTranslateMatrix(translate);

	Matrix4x4 rotateMatrix = Multiply(rotateXMatrix, Multiply(rotateYMatrix, rotateZMatrix)); // Rx * Ry * Rz
	Matrix4x4 result = Multiply(scaleMatrix, rotateMatrix);
	result = Multiply(result, translateMatrix); // S * R * T
	return result;
}

/// <summary>
/// 逆行列を計算 (回転と平行移動のみで構成されるアフィン変換行列用)
/// </summary>
Matrix4x4 Inverse(const Matrix4x4& matrix) {
	Matrix4x4 result = MakeIdentity4x4();
	// 回転部分の転置
	result.m[0][0] = matrix.m[0][0]; result.m[0][1] = matrix.m[1][0]; result.m[0][2] = matrix.m[2][0];
	result.m[1][0] = matrix.m[0][1]; result.m[1][1] = matrix.m[1][1]; result.m[1][2] = matrix.m[2][1];
	result.m[2][0] = matrix.m[0][2]; result.m[2][1] = matrix.m[1][2]; result.m[2][2] = matrix.m[2][2];
	// 平行移動部分の計算: t' = - (t * R^T) (行ベクトルとして)
	Vector3 translate = { matrix.m[3][0], matrix.m[3][1], matrix.m[3][2] };
	result.m[3][0] = -(translate.x * result.m[0][0] + translate.y * result.m[1][0] + translate.z * result.m[2][0]);
	result.m[3][1] = -(translate.x * result.m[0][1] + translate.y * result.m[1][1] + translate.z * result.m[2][1]);
	result.m[3][2] = -(translate.x * result.m[0][2] + translate.y * result.m[1][2] + translate.z * result.m[2][2]);
	return result;
}

/// <summary>
/// 透視投影行列を作成する関数
/// </summary>
Matrix4x4 MakePerspectiveFovMatrix(float fovY, float aspectRatio, float nearClip, float farClip) {
	Matrix4x4 result = {}; // ゼロ初期化
	float cotHalfFovY = 1.0f / std::tan(fovY / 2.0f);
	result.m[0][0] = cotHalfFovY / aspectRatio;
	result.m[1][1] = cotHalfFovY;
	result.m[2][2] = farClip / (farClip - nearClip);
	result.m[2][3] = 1.0f;
	result.m[3][2] = (-nearClip * farClip) / (farClip - nearClip);
	return result;
}

/// <summary>
/// ビューポート変換行列を作成する関数
/// </summary>
Matrix4x4 MakeViewportMatrix(float left, float top, float width, float height, float minDepth, float maxDepth) {
	Matrix4x4 result = MakeIdentity4x4();
	result.m[0][0] = width / 2.0f;
	result.m[1][1] = -height / 2.0f; // Y軸反転
	result.m[2][2] = maxDepth - minDepth;
	result.m[3][0] = left + (width / 2.0f);
	result.m[3][1] = top + (height / 2.0f);
	result.m[3][2] = minDepth;
	return result;
}

/// <summary>
/// ベクトルを4x4行列で変換 (w除算あり、行ベクトル v' = v * M)
/// </summary>
Vector3 Transform(const Vector3& vector, const Matrix4x4& matrix) {
	Vector3 result;
	float w = vector.x * matrix.m[0][3] + vector.y * matrix.m[1][3] + vector.z * matrix.m[2][3] + 1.0f * matrix.m[3][3];
	if (std::abs(w) < 1e-6f) {
		w = 1.0f;
	}
	result.x = (vector.x * matrix.m[0][0] + vector.y * matrix.m[1][0] + vector.z * matrix.m[2][0] + 1.0f * matrix.m[3][0]) / w;
	result.y = (vector.x * matrix.m[0][1] + vector.y * matrix.m[1][1] + vector.z * matrix.m[2][1] + 1.0f * matrix.m[3][1]) / w;
	result.z = (vector.x * matrix.m[0][2] + vector.y * matrix.m[1][2] + vector.z * matrix.m[2][2] + 1.0f * matrix.m[3][2]) / w;
	return result;
}

/// <summary>
/// グリッドをワイヤーフレームで描画
/// </summary>
void DrawGrid(const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix) {
	const float kGridHalfWidth = 2.0f;
	const uint32_t kSubdivision = 10;
	const float kGridEvery = (kGridHalfWidth * 2.0f) / static_cast<float>(kSubdivision);
	uint32_t gridColor = 0xAAAAAAFF;
	Matrix4x4 worldMatrix = MakeIdentity4x4();

	Matrix4x4 worldViewProjectionViewportMatrix = Multiply(worldMatrix, Multiply(viewProjectionMatrix, viewportMatrix));

	for (uint32_t xIndex = 0; xIndex <= kSubdivision; ++xIndex) {
		float x = -kGridHalfWidth + (kGridEvery * xIndex);
		Vector3 start = { x, 0.0f, -kGridHalfWidth };
		Vector3 end = { x, 0.0f, kGridHalfWidth };

		Vector3 screenStart = Transform(start, worldViewProjectionViewportMatrix);
		Vector3 screenEnd = Transform(end, worldViewProjectionViewportMatrix);

		Novice::DrawLine(static_cast<int>(screenStart.x), static_cast<int>(screenStart.y), static_cast<int>(screenEnd.x), static_cast<int>(screenEnd.y), gridColor);
	}
	for (uint32_t zIndex = 0; zIndex <= kSubdivision; ++zIndex) {
		float z = -kGridHalfWidth + (kGridEvery * zIndex);
		Vector3 start = { -kGridHalfWidth, 0.0f, z };
		Vector3 end = { kGridHalfWidth, 0.0f, z };

		Vector3 screenStart = Transform(start, worldViewProjectionViewportMatrix);
		Vector3 screenEnd = Transform(end, worldViewProjectionViewportMatrix);
		Novice::DrawLine(static_cast<int>(screenStart.x), static_cast<int>(screenStart.y), static_cast<int>(screenEnd.x), static_cast<int>(screenEnd.y), gridColor);
	}
}

/// <summary>
/// 球をワイヤーフレームで描画
/// </summary>
void DrawSphere(const Vector3& centerWorld, float radiusWorld, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color) {
	const uint32_t kSubdivision = 12;
	float latEvery = static_cast<float>(M_PI) / kSubdivision;
	float lonEvery = 2.0f * static_cast<float>(M_PI) / kSubdivision;

	Matrix4x4 sphereWorldMatrix = MakeTranslateMatrix(centerWorld);
	Matrix4x4 worldViewProjectionViewportMatrix = Multiply(sphereWorldMatrix, Multiply(viewProjectionMatrix, viewportMatrix));

	for (uint32_t latIndex = 0; latIndex < kSubdivision; ++latIndex) {
		float lat = -static_cast<float>(M_PI) / 2.0f + latEvery * latIndex;
		float latNext = lat + latEvery;

		for (uint32_t lonIndex = 0; lonIndex < kSubdivision; ++lonIndex) {
			float lon = lonEvery * lonIndex;
			float lonNext = lon + lonEvery;

			Vector3 p0_local = { std::cos(lat) * std::cos(lon) * radiusWorld, std::sin(lat) * radiusWorld, std::cos(lat) * std::sin(lon) * radiusWorld };
			Vector3 p1_local = { std::cos(lat) * std::cos(lonNext) * radiusWorld, std::sin(lat) * radiusWorld, std::cos(lat) * std::sin(lonNext) * radiusWorld };
			Vector3 p2_local = { std::cos(latNext) * std::cos(lon) * radiusWorld, std::sin(latNext) * radiusWorld, std::cos(latNext) * std::sin(lon) * radiusWorld };

			Vector3 screenP0 = Transform(p0_local, worldViewProjectionViewportMatrix);
			Vector3 screenP1 = Transform(p1_local, worldViewProjectionViewportMatrix);
			Vector3 screenP2 = Transform(p2_local, worldViewProjectionViewportMatrix);

			Novice::DrawLine(static_cast<int>(screenP0.x), static_cast<int>(screenP0.y), static_cast<int>(screenP1.x), static_cast<int>(screenP1.y), color);
			Novice::DrawLine(static_cast<int>(screenP0.x), static_cast<int>(screenP0.y), static_cast<int>(screenP2.x), static_cast<int>(screenP2.y), color);
		}
	}
}


// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {
	Novice::Initialize(kWindowTitle, static_cast<int>(kWindowWidth), static_cast<int>(kWindowHeight));

	char keys[256] = { 0 };
	char preKeys[256] = { 0 };

	// カメラ設定
	Vector3 cameraTranslate{ 0.0f, 1.9f, -6.49f };
	Vector3 cameraRotate{ 0.26f, 0.0f, 0.0f };
	float fovY = 0.45f;
	float aspectRatio = kWindowWidth / kWindowHeight;
	float nearClip = 0.1f;
	float farClip = 100.0f;

	const float deltaTime = 1.0f / 60.0f; // 固定タイムステップ (秒)

	// ばねとボールの初期化
	Spring spring;
	spring.anchor = { 0.0f, 0.8f, 0.0f };
	spring.naturalLength = 1.0f;
	spring.stiffness = 100.0f;
	spring.dampingCoefficient = 2.0f;

	Ball ball;
	ball.position = { 1.2f, 0.8f, 0.0f };
	ball.velocity = { 0.0f, 0.0f, 0.0f };
	ball.mass = 2.0f;
	ball.radius = 0.05f;
	ball.color = 0x0000FFFF; // 青色

	uint32_t anchorColor = 0xFF0000FF;
	float anchorRadius = 0.02f;
	uint32_t springLineColor = 0xFFFFFFFF;


	while (Novice::ProcessMessage() == 0) {
		Novice::BeginFrame();
		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		// --- 更新処理 ---
		Matrix4x4 cameraMatrix = MakeAffineMatrix({ 1.0f, 1.0f, 1.0f }, cameraRotate, cameraTranslate);
		Matrix4x4 viewMatrix = Inverse(cameraMatrix);
		Matrix4x4 projectionMatrix = MakePerspectiveFovMatrix(fovY, aspectRatio, nearClip, farClip);
		Matrix4x4 viewProjectionMatrix = Multiply(viewMatrix, projectionMatrix);
		Matrix4x4 viewportMatrix = MakeViewportMatrix(0.0f, 0.0f, kWindowWidth, kWindowHeight, 0.0f, 1.0f);

		// ボールの物理演算
		Vector3 displacement = Subtract(ball.position, spring.anchor);
		float currentLength = Length(displacement);
		Vector3 direction = Normalize(displacement);

		// ばねの力
		Vector3 springForce = { 0.0f, 0.0f, 0.0f };
		if (currentLength != 0) {
			springForce = MultiplyScalar(direction, -spring.stiffness * (currentLength - spring.naturalLength));
		}

		// 減衰力
		Vector3 dampingForce = MultiplyScalar(ball.velocity, -spring.dampingCoefficient);

		// 合力
		Vector3 totalForce = Add(springForce, dampingForce);

		// 加速度
		Vector3 acceleration = MultiplyScalar(totalForce, 1.0f / ball.mass);

		// 速度の更新
		ball.velocity = Add(ball.velocity, MultiplyScalar(acceleration, deltaTime));

		// 位置の更新
		ball.position = Add(ball.position, MultiplyScalar(ball.velocity, deltaTime));


		// --- ImGui ---
		ImGui::Begin("Spring Settings");
		ImGui::DragFloat3("Anchor Position", &spring.anchor.x, 0.01f);
		ImGui::DragFloat("Natural Length", &spring.naturalLength, 0.01f, 0.1f, 5.0f);
		ImGui::DragFloat("Stiffness", &spring.stiffness, 0.1f, 0.0f, 500.0f);
		ImGui::DragFloat("Damping", &spring.dampingCoefficient, 0.01f, 0.0f, 20.0f);
		ImGui::Separator();
		ImGui::DragFloat("Ball Mass", &ball.mass, 0.01f, 0.1f, 10.0f);
		ImGui::Text("Ball Position: %.2f, %.2f, %.2f", ball.position.x, ball.position.y, ball.position.z);
		ImGui::Text("Ball Velocity: %.2f, %.2f, %.2f", ball.velocity.x, ball.velocity.y, ball.velocity.z);
		ImGui::End();

		// --- 描画処理 ---
		DrawGrid(viewProjectionMatrix, viewportMatrix);

		// アンカーを描画
		DrawSphere(spring.anchor, anchorRadius, viewProjectionMatrix, viewportMatrix, anchorColor);

		// ボールを描画
		DrawSphere(ball.position, ball.radius, viewProjectionMatrix, viewportMatrix, ball.color);

		// ばね（線）を描画
		Vector3 screenAnchor = Transform(Transform(spring.anchor, viewProjectionMatrix), viewportMatrix);
		Vector3 screenBall = Transform(Transform(ball.position, viewProjectionMatrix), viewportMatrix);
		Novice::DrawLine(
			static_cast<int>(screenAnchor.x), static_cast<int>(screenAnchor.y),
			static_cast<int>(screenBall.x), static_cast<int>(screenBall.y),
			springLineColor);


		Novice::EndFrame();
		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
			break;
		}
	}

	Novice::Finalize();
	return 0;
}